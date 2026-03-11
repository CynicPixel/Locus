# MLAT Implementation Fixes — Cross-Reference with mlat-server

**Source**: Detailed comparison of `rust-backend/src/` against `mlat-server/` (Oliver Jowett's production MLAT system)  
**Date**: 2026-03-12  
**Goal**: Identify theoretical bugs, missing techniques, and algorithm improvements

---

## CRITICAL — Theoretical Bugs That Affect Position Accuracy

### FIX-1: Speed of Light — Vacuum vs Air (ALL TDOA calculations wrong)

| | mlat-server | Locus Rust |
|---|---|---|
| **Value** | `Cair = 299792458 / 1.0003` = **299,702,547 m/s** | `C_M_PER_NS = 0.299_792_458` = **299,792,458 m/s** |
| **What it is** | Speed of light **in air** (accounts for atmospheric refractive index n≈1.0003) | Speed of light **in vacuum** |

**Impact**: Mode S signals travel through the atmosphere, not vacuum. Using c_vacuum instead of c_air introduces a **0.03% systematic TDOA error**. At 300 km receiver range, this is **~90 m error per pseudorange**. At shorter ranges like 50 km it's ~15 m — still significant for MLAT.

**Files to change**:
- `rust-backend/src/main.rs` line 53: `C_M_PER_NS`
- `rust-backend/src/clock_sync.rs` line 10: `C_M_PER_NS`

**Fix**:
```rust
// BEFORE (vacuum):
const C_M_PER_NS: f64 = 0.299_792_458;
// AFTER (air, matching mlat-server):
const C_M_PER_NS: f64 = 0.299_702_547;  // 299_792_458 / 1.0003 / 1e9
```

---

### FIX-2: Solver Formulation — Missing 4th Unknown (Range Offset)

**mlat-server** solves for **4 unknowns**: `[x, y, z, offset]` where `offset` is the pseudorange to the reference receiver. Each residual is:

```python
residual = (measured_pseudorange - (distance_to_receiver - offset)) / error
```

**Locus** solves for **3 unknowns**: `[x, y, z]` using pure TDOAs. Each residual is:

```rust
residual = observed_tdoa - (dist_k - ref_dist)
```

**Why the 4-variable formulation is superior**:
1. **No privileged reference receiver** — all receivers contribute symmetrically. In the 3-var TDOA formulation, `sensor[0]` has special status and its noise doesn't appear in the residuals.
2. **Absorbs residual clock biases** — the `offset` variable absorbs any remaining clock synchronization error. In Locus, if clock_sync hasn't converged perfectly, that error goes straight into the position estimate.
3. **Better numerical conditioning** — differencing two large, nearly-equal distances (`dist_k - ref_dist`) causes catastrophic cancellation. The pseudorange formulation avoids this.

**Files to change**: `rust-backend/src/mlat_solver.rs` — restructure `MlatProblem` to have 4 parameters `[x, y, z, offset]`.

---

### FIX-3: Altitude Constraint Uses Spherical Earth (21 km systematic error)

**mlat-server**: Altitude observation uses actual WGS84 altitude via `ecef2llh()`:
```python
_, _, altitude_guess = geodesy.ecef2llh(position_guess)
residual = (altitude - altitude_guess) / altitude_error
```

**Locus**: Uses a spherical norm constraint:
```rust
let alt_constraint_r = input.adsb_alt_m.map(|a| R_EARTH_M + a);  // R_EARTH_M = 6_371_000
residual = weight * (|p| - r_constraint)
```

**Impact**: The WGS84 ellipsoid radius varies from 6,357 km (poles) to 6,378 km (equator). Using `R = 6,371 km` introduces **up to ±7 km error in radius**, which translates to a **7 km systematic altitude bias** depending on latitude. At 45°N (typical for UK/Europe MLAT), the error is ~10 km. The altitude constraint is essentially pointing the solver at a wrong altitude shell.

**Fix**: Replace the spherical norm constraint with the proper WGS84 altitude computation:
```rust
// In residuals():
if let Some(target_alt) = self.alt_constraint_m {
    let (_, _, alt_guess) = ecef_to_wgs84(self.p[0], self.p[1], self.p[2]);
    r.push(self.alt_weight * (alt_guess - target_alt));
}
```

**File**: `rust-backend/src/mlat_solver.rs` — `MlatProblem::residuals()` and `jacobian()`

---

### FIX-4: No Per-Measurement Error Weighting in Solver

**mlat-server**: Each residual is divided by `error = sqrt(variance) * Cair`:
```python
pseudorange_data = [(pos, (ts - base_ts) * Cair, sqrt(variance) * Cair) for ...]
residual = (pseudorange - pseudorange_guess) / error
```

**Locus**: All residuals have equal weight:
```rust
residual = obs - (dist_k - ref_dist)  // no division by error
```

**Impact**: Noisy receivers (poor clocks, multipath) contribute equally to good receivers. This degrades position accuracy when receiver quality varies.

**Fix**: Pass clock pairing variance into the solver. Each TDOA residual should be divided by `sqrt(variance_i + variance_ref) * C_air`. This requires `clock_sync` to export variance estimates.

---

### FIX-5: No CRC-24 Verification on Mode S Messages

**mlat-server**: Full CRC-24 verification — messages with bad CRC are rejected:
```python
self.crc_ok = (crc.residual(frombuf) == 0)
```

**Locus**: No CRC check at all. Only checks DF byte value:
```rust
let df = (bytes[0] >> 3) & 0x1F;
if df != 17 && df != 18 { return None; }
```

**Impact**: Corrupted messages with bit errors can produce:
1. Wrong ICAO addresses → frames grouped with wrong aircraft
2. Wrong CPR lat/lon values → bad beacon positions contaminating clock sync
3. Wrong altitude values → bad altitude constraints

**Fix**: Implement CRC-24/Mode S in `adsb_parser.rs`. The generator polynomial is `0x1FFF409`. A message is valid when the CRC remainder is zero (for DF17/18).

---

## HIGH PRIORITY — Missing Algorithms That Significantly Affect Quality

### FIX-6: Clock Sync Has No Drift Tracking (clocks diverge between observations)

**mlat-server** (`clocksync.py`): Tracks both clock **offset** and clock **drift** (frequency error) using a PI controller:
```python
# Drift estimated from interval ratio:
new_drift = (peer_interval - adjusted_base_interval) / adjusted_base_interval

# PI controller:
self.raw_drift += drift_error * self.KP  # Proportional
self.drift = self.raw_drift - self.KI * self.cumulative_error  # Integral

# Prediction uses drift:
prediction = ts_peer[-1] + elapsed * relative_freq * (1 + drift)
```

**mlat-server** uses **TWO messages** per sync point (even/odd CPR pair). The interval between them lets it compute the drift directly from the ratio of receiver intervals.

**Locus** (`clock_sync.rs`): Only stores offset samples and takes the median. No drift estimation at all:
```rust
// Just stores error_ns values, returns median.
// Between observations, no prediction of how the offset is changing.
```

**Impact**: Real receiver clocks drift. A 1 PPM frequency error accumulates 1 µs per second. With Locus's 50-sample window and ~10 observations/second, the oldest sample is 5 seconds stale. A 5 PPM clock (typical for Beast receivers) drifts 25 µs in 5 seconds = **7.5 km TDOA error**. Without drift tracking, the median will be contaminated by stale, drifted samples.

**Fix**: Implement drift tracking. Options:
1. Port mlat-server's PI controller approach (KP=0.05, KI=0.01)
2. Use message pairs (even+odd) for interval-based drift estimation
3. At minimum, use linear regression on the offset samples instead of median

**File**: `rust-backend/src/clock_sync.rs` — major restructure of `PairState`

---

### FIX-7: Clock Normalization — No MST Graph (error accumulates arbitrarily)

**mlat-server** (`clocknorm.py`): Before solving, normalizes all timestamps to a common reference by:
1. Building a weighted graph of receiver pairs (edge weight = pairing variance)
2. Computing the Minimum Spanning Tree (minimizes total conversion error)
3. Picking a **central node** (minimizes the maximum path cost)
4. Propagating timestamps through the tree via piecewise linear interpolation

**Locus** (`main.rs`): Uses `frames[0]` as the reference and directly subtracts offsets:
```rust
clock_sync.apply_corrections(&mut group.frames);
// frames[0] is reference; frames[1..] get offset subtracted
```

**Impact**: If you have receivers A, B, C and the pair (A,C) has poor sync quality but (A,B) and (B,C) are good, Locus will use the bad (A,C) pairing directly instead of going through B. The MST approach ensures the lowest-error conversion path is always used.

**Fix**: Implement a graph-based clock normalization:
1. For each group, build a graph of available pairings with variance weights
2. Find the MST
3. Pick the central node
4. Convert all timestamps to the central clock through the tree

**Files**: New module or extension to `clock_sync.rs`

---

### FIX-8: Kalman Filter — EKF vs UKF and Wrong Process Noise

**mlat-server** (`kalman.py`): Uses an **Unscented Kalman Filter (UKF)** with:
- **9-state Constant Acceleration** model: `[x, y, z, vx, vy, vz, ax, ay, az]`
- Process noise: `σ_a = 0.10 m/s²` (realistic for commercial aviation)
- Initial velocity uncertainty: `200 m/s`
- **Direct TDOA observation function**: the UKF computes pseudoranges from the predicted state, so the full nonlinear observation is handled by the unscented transform
- **Mahalanobis outlier detection**: rejects observations with M-distance > 15

**Locus** (`kalman.rs`): Uses a **6-state EKF** with linear observation:
- **6-state Constant Velocity** model: `[x, y, z, vx, vy, vz]`
- Process noise: `σ_a² = 2500` → `σ_a = 50 m/s²` (≈5g — fighter jet level, not commercial aviation)
- Initial velocity uncertainty: `50 m/s`
- Observation is a **position update from the solver** with linear H = `[I₃ 0₃]`
- No outlier detection

**Specific issues**:

1. **Process noise is ~500× too high**: `σ_a = 50 m/s²` vs mlat-server's `0.10 m/s²`. This makes the Kalman filter essentially trust measurements over predictions — it barely smooths. For commercial aircraft, `0.5–2.0 m/s²` is reasonable (matching mlat-server's CV model `accel_noise = 0.5`).

2. **No acceleration state**: The CV model cannot track turning or climbing/descending aircraft well. The CA model in mlat-server handles maneuvers.

3. **No Mahalanobis outlier gating**: A single bad MLAT solve (which happens) can corrupt the Kalman state. mlat-server rejects observations with M-distance > 15 and resets after 3 consecutive outliers.

4. **Two-stage pipeline loses information**: Locus runs solver → Kalman as separate stages. The solver's residual covariance is approximated as `accuracy_m²` instead of being properly propagated. mlat-server folds the raw TDOA observations directly into the UKF observation function, preserving full information.

**Priority fixes** (in order of impact):
1. Reduce `sigma_a2` from `2500.0` to `1.0` (σ_a = 1 m/s², good for commercial aviation)
2. Add Mahalanobis outlier gating before filter update
3. Add acquiring/tracking state machine
4. Consider 9-state CA model for better maneuver tracking

**File**: `rust-backend/src/kalman.rs`

---

### FIX-9: Only DF17/18 Messages Used for MLAT (misses majority of usable signals)

**mlat-server**: Correlates **ALL Mode S message types** — DF0, DF4, DF5, DF11, DF16, DF17, DF18, DF20, DF21. Any message received by 3+ receivers can be multilaterated.

**Locus** (`correlator.rs`):
```rust
let df = (bytes[0] >> 3) & 0x1F;
if df != 17 && df != 18 { return None; }
```

**Impact**: Most aircraft transponders produce DF4/5/11/20/21 replies to radar interrogations at a much higher rate than DF17 extended squitters. By restricting to DF17/18 only, Locus ignores the majority of multilaterable messages. This dramatically reduces:
- Position update rate
- Coverage (some aircraft don't transmit ADS-B at all but still reply to interrogations)
- Clock sync opportunities

**Fix**: Accept all message types for correlation. Non-DF17/18 messages don't carry position info, but they CAN still be correlated and multilaterated — you just need ≥4 receivers (since there's no altitude constraint).

**File**: `rust-backend/src/correlator.rs` — `CorrelationKey::from_frame()`

---

### FIX-10: No NUC/NIC Quality Gate on Sync Beacons

**mlat-server** requires NUC ≥ 6 for clock sync source aircraft:
```python
if even_message.nuc < 6 or even_message.altitude is None:
    return  # reject this sync pair
```

**Locus**: Uses any decoded ADS-B position as a beacon, regardless of quality.

**Impact**: NUC < 6 means position uncertainty > 926 m. Using an inaccurate aircraft position to calibrate clock offsets contaminates the clock sync with hundreds of meters of error.

**Fix**: Check TC (type code) to derive NUC/NIC and reject sync beacons with NUC < 6.

**File**: `rust-backend/src/main.rs` — the `solve_and_broadcast` function where `update_from_beacon` is called.

---

## MEDIUM PRIORITY — Better Techniques to Adopt

### FIX-11: No Sync Pair Expiry or Validity Gating

**mlat-server**: Clock pairings have:
- **Expiry**: 120s without an update → pairing deleted
- **Validity window**: 30s since last sync → pairing becomes invalid
- **Quality threshold**: requires `n ≥ 2` and `variance < 16e-12` (4 µs RMS error)
- **Outlier counter**: 5 consecutive outliers → discard

**Locus**: No expiry, no validity check, no quality threshold. Old pairings with stale offset estimates are used forever.

**Fix**: Add to `PairState`:
- A `last_update_ns` field
- Consider pairings invalid if not updated in 30 seconds
- Require minimum sample count before using for corrections

**File**: `rust-backend/src/clock_sync.rs`

---

### FIX-12: Clock Sync Uses Piecewise Linear Interpolation (not just point estimates)

**mlat-server** (`clocksync.py`): `predict_peer()` uses **piecewise linear interpolation** between stored sync points:
```python
# Interpolate between two points:
return (self.ts_peer[i-1] +
        (self.ts_peer[i] - self.ts_peer[i-1]) *
        (base_ts - self.ts_base[i-1]) /
        (self.ts_base[i] - self.ts_base[i-1]))
```

This means for any arbitrary timestamp, it interpolates between the nearest two sync points, accounting for clock drift between them.

**Locus**: Uses a single median offset for each pair. This is a zero-order hold — the correction is a constant that steps when the median changes.

**Impact**: Between sync observations, the true offset drifts linearly. Interpolation captures this; a constant offset does not.

---

### FIX-13: Timestamp Feasibility Checking in Correlator

**mlat-server** (`mlattrack.py`): Before clustering measurements, checks physical feasibility:
```python
d = receiver.distance[other_receiver]
if abs(other_timestamp - timestamp) > (d * 1.05 + 1e3) / constants.Cair:
    can_cluster = False  # timestamps incompatible with receiver geometry
```

**Locus**: No such check. Any frames with the same content_hash within 50 ms are grouped together, even if the timestamp differences are physically impossible.

**Impact**: If two receivers are 100 km apart, the maximum possible timestamp difference is ~333 µs. If the observed difference is larger, the frame is likely corrupt or mismatched. Without this check, bad frames enter the solver.

**Fix**: After correlation, before solving, validate that `|t_i - t_j| ≤ d(receiver_i, receiver_j) * 1.05 / C_air + guard`.

**File**: `rust-backend/src/correlator.rs` or at the start of `solve_and_broadcast` in `main.rs`

---

### FIX-14: Rate Limiting Per-Aircraft Solutions

**mlat-server**: Limits MLAT solutions per aircraft:
- Minimum 2.0s between solutions at the same DOF
- 10s minimum at reduced DOF
- Also skips solutions less accurate than the recent result (variance comparison)

**Locus**: Emits a solution for every correlated group (every 50ms eviction cycle). At 10-20 beacons per second, the same aircraft can produce 10+ solutions per second.

**Impact**: Excessive solutions waste CPU and can cause the Kalman filter to over-update with correlated measurements (multiple solutions from overlapping message groups).

**Fix**: Add a per-aircraft rate limiter (e.g., skip if last solve < 1-2 seconds ago and DOF hasn't improved).

**File**: `rust-backend/src/main.rs` — `solve_and_broadcast()`

---

### FIX-15: Post-Solve Range Validation

**mlat-server**: After solving, validates the result:
```python
if offset_est < 0 or offset_est > MAX_RANGE:
    return None  # implausible
for receiver in measurements:
    if distance(receiver, position_est) > MAX_RANGE:
        return None  # too far
```
Also rejects solutions with `trace(cov) > 100e6` (>10 km uncertainty).

**Locus**: Only checks `GDOP > 10`. No range validation, no covariance magnitude check.

**Fix**: After solving, verify:
1. Solved position is within MAX_RANGE (500 km) of all receivers
2. Covariance trace is sane (< 100e6 = 10 km²)

**File**: `rust-backend/src/mlat_solver.rs` — `solve()`

---

### FIX-16: Altitude Error Degrades Over Time

**mlat-server**: Altitude accuracy degrades with age:
```python
if decoded.altitude is not None:
    altitude_error = 250 * FTOM  # fresh: ~76m
elif altitude is not None:
    # Degrade at ~4000 fpm (70 ft/s)
    altitude_error = (250 + (cluster_utc - ac.last_altitude_time) * 70) * FTOM
```

**Locus**: Uses a fixed 100m weight for altitude constraint, regardless of age.

**Impact**: If an aircraft is climbing/descending at 2000 fpm (~10 m/s), a 5-second-old altitude report is wrong by ~50 m. A 30-second-old report by ~300 m. Using stale altitude with tight constraints sends the solver to the wrong place.

**Fix**: Track `last_altitude_time` per aircraft. Degrade altitude constraint weight:
```rust
let alt_error_m = 76.0 + (age_seconds * 21.3);  // 250ft + 70ft/s * age
let alt_weight = 1.0 / alt_error_m;
```

---

### FIX-17: Solver `maxfev` / Patience is Too High

**mlat-server**: `SOLVER_MAXFEV = 50` (max function evaluations).

**Locus**: `LevenbergMarquardt::new().with_patience(100)`.

**Impact**: If the solver takes > 50 iterations, the geometry is probably too poor for a good solution anyway. Reducing patience to 50 saves CPU on hopeless cases.

---

## LOW PRIORITY — Nice-to-Have Improvements

### FIX-18: Gillham Gray Code Altitude Decoding

**mlat-server** (`altitude.py`): Fully decodes **both** Q-bit (25 ft) and Gillham Gray code (100 ft) altitude encodings.

**Locus** (`adsb_parser.rs`):
```rust
if q_bit == 1 { /* 25 ft decode */ }
else { None }  // Gillham not implemented
```

**Impact**: Older transponders use Gillham code (Q-bit=0). These frames are discarded, losing altitude data that could constrain the solver.

---

### FIX-19: Correlation Window — 50 ms May Be Too Short

**mlat-server**: Uses `MLAT_DELAY = 2.5 seconds`.

**Locus**: Uses a 50 ms eviction window.

**Note**: These serve slightly different purposes — mlat-server accumulates over the network (receivers report asynchronously), while Locus receives pre-timestamped frames. However, 50 ms may still be too tight if:
1. Distant receivers have high latency
2. Frames arrive out of order

Consider increasing to 100-500 ms or making it configurable.

---

### FIX-20: Clock Type Awareness

**mlat-server**: Handles different receiver clocks with explicit parameters:
```python
'radarcape_gps': Clock(epoch='gps_midnight', freq=1e9, max_freq_error=1e-6, jitter=15e-9)
'beast':         Clock(epoch=None, freq=12e6, max_freq_error=5e-6, jitter=83e-9)
'sbs':           Clock(epoch=None, freq=20e6, max_freq_error=100e-6, jitter=500e-9)
```

**Locus**: Assumes all timestamps are in nanoseconds with uniform quality.

**Impact**: Different receiver hardware has vastly different clock quality. A Radarcape GPS has 15 ns jitter while an SBS has 500 ns. Without clock type awareness, Locus can't properly weight measurements.

---

### FIX-21: Degrees of Freedom (DOF) Tracking

**mlat-server**: Tracks `DOF = n_receivers + (1 if altitude) - 4` per solution.

**Locus**: Has no concept of DOF.

DOF is useful for:
1. Rate limiting (require higher DOF for more frequent updates)
2. Deciding whether altitude is needed (DOF < 0 → not enough data)
3. Quality reporting

---

### FIX-22: Distance-Based Clustering Within Correlation Groups

**mlat-server** (`mlattrack.py`): After clock normalization, clusters frames into sub-groups where co-visible receivers have compatible timestamps (within `d * 1.05 / Cair`). Receivers that are far apart and see the same message through different propagation paths get separated.

**Locus**: All frames in a correlation group with the same content_hash are treated as one observation set.

---

## SUMMARY — Prioritized Fix Order

| Priority | Fix | Effort | Impact on Accuracy |
|----------|-----|--------|-------------------|
| **P0** | FIX-1: Speed of light in air vs vacuum | 5 min | ~90m systematic error removed |
| **P0** | FIX-3: WGS84 altitude constraint (not spherical) | 30 min | Up to 7-21 km altitude bias removed |
| **P0** | FIX-5: CRC-24 verification | 1 hr | Prevents corrupted data entering pipeline |
| **P1** | FIX-2: 4-variable pseudorange solver | 2 hr | Better conditioning, absorbs clock bias |
| **P1** | FIX-8: Kalman process noise (50→1 m/s²) | 5 min | Filter actually smooths instead of passing through |
| **P1** | FIX-8: Mahalanobis outlier gating in Kalman | 1 hr | Prevents bad solves from corrupting filter |
| **P1** | FIX-4: Per-measurement error weighting | 1 hr | Noisy receivers down-weighted |
| **P2** | FIX-6: Clock drift tracking | 3 hr | Corrects for receiver frequency errors |
| **P2** | FIX-9: Accept all DF types for correlation | 1 hr | 3-10× more data points |
| **P2** | FIX-10: NUC ≥ 6 gate on sync beacons | 15 min | Prevents bad positions in clock cal |
| **P2** | FIX-13: Timestamp feasibility check | 30 min | Rejects impossible observations |
| **P2** | FIX-15: Post-solve range validation | 15 min | Catches wildly wrong solutions |
| **P3** | FIX-7: MST clock normalization | 3 hr | Optimal clock conversion paths |
| **P3** | FIX-11: Sync pair expiry/validity | 30 min | Prevents stale clock data |
| **P3** | FIX-14: Per-aircraft rate limiting | 30 min | Reduces redundant updates |
| **P3** | FIX-16: Altitude error degradation | 15 min | Time-aware altitude weighting |
| **P4** | FIX-12: Piecewise linear interpolation | 2 hr | Smoother clock corrections |
| **P4** | FIX-18: Gillham altitude decode | 1 hr | Support older transponders |
| **P4** | FIX-19: Larger correlation window | 5 min | Catch late-arriving frames |
| **P4** | FIX-20: Clock type awareness | 1 hr | Proper per-receiver error modeling |
| **P4** | FIX-21: DOF tracking | 30 min | Better solution quality control |
| **P4** | FIX-22: Distance-based sub-clustering | 1 hr | Separates multipath from signal |

---

## Appendix: What Locus Does BETTER Than mlat-server

For completeness, areas where the Locus implementation is already strong:

1. **Geometry pre-check (SVD/cache)**: Locus pre-computes sensor geometry quality via SVD and caches it — mlat-server has no equivalent pre-solve geometry check.
2. **GDOP computation**: Locus computes actual GDOP from `(J^T J)^{-1}` — mlat-server uses `trace(cov)` as a simpler proxy.
3. **Confidence ellipse**: Locus projects covariance to ENU and extracts the semi-major axis — mlat-server doesn't compute this.
4. **Spoof detection**: Locus has a GDOP-adaptive spoof detector comparing MLAT vs ADS-B — absent from mlat-server.
5. **ECEF sensor caching**: Locus caches ECEF conversions per sensor — mlat-server recomputes per message.
6. **Initial guess**: Locus's centroid-at-altitude initial guess is better than mlat-server's fallback to receiver position.
7. **Real-time WebSocket output**: Locus streams positions via WebSocket — mlat-server uses batch file output.
8. **AI anomaly detection**: The LSTM classifier layer is entirely absent from mlat-server.