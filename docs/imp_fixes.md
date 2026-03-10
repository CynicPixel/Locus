KalmanStatePoint in Anomaly Client Uses Degrees, Model Trained in Degrees — Mismatch With ECEF Kalman
Location: Lines 2387–2393 and 2469–2472
The KalmanStatePoint sent to the ML service is defined as:
pythonclass KalmanStatePoint(BaseModel):
    lat: float
    lon: float
    alt_m: float
    vel_lat: float
    vel_lon: float
    vel_alt: float
And the training data in generate_synthetic_dataset() uses degrees for lat/lon. The model is trained on degree-space inputs.
But your Kalman filter now runs in ECEF. The velocity state (vx, vy, vz) is in ECEF m/s. When you populate the VecDeque<(f64,f64,f64,f64,f64,f64)> history at line 2519, you'll be pulling velocity from the ECEF state — values like (200.0, -150.0, 50.0) m/s in ECEF. The model expects velocity in degrees/second — values like (0.002, 0.004, -1.0).
You have two mismatches:

Velocity units: ECEF m/s vs training data degrees/s
Velocity magnitude: ECEF velocities are ~200 m/s; training velocities are ~0.003 deg/s

This won't crash anything but the model will receive inputs so far outside its training distribution that every aircraft will score near 50% confidence and labels will be meaningless.
Fix: Either convert ECEF velocity to degrees/s before sending to the ML service, or retrain the model on ECEF inputs. The simpler fix is conversion at the VecDeque population site:
rust// When adding to history, convert ECEF velocity to approx degrees/s
let (lat, lon, _) = entry.position_wgs84();
let (vx, vy, vz) = entry.velocity_ms();
// Approximate conversion (good enough for anomaly detection)
let vel_lat = vz / 111_320.0;  // m/s to deg/s via pole direction
let vel_lon = (vx * lon.to_radians().cos() + vy * lon.to_radians().sin()) 
              / (111_320.0 * lat.to_radians().cos());
history.push_back((lat, lon, alt_m, vel_lat, vel_lon, vel_alt_ms));
Or just accept that the ML component is cosmetic and document it clearly. But don't silently pass ECEF velocities to a degrees-trained model and call the output meaningful.


1. The Correlator Emits On min_sensors — This Is Wrong For MLAT
This is the most serious conceptual error that survived both versions.
The correlator emits a group the moment frames.len() >= min_sensors (set to 3). So with 8 sensors deployed, the first 3 sensor packets that arrive for a given message trigger an immediate solve. Sensors 4 through 8 arrive milliseconds later, get assigned to the same key, find the group already removed from the HashMap, and are silently discarded.
The problem is that more sensors always means better geometry and lower GDOP. Three sensors in a line give GDOP of infinity — the system is geometrically degenerate, you cannot solve for position. Four sensors might give GDOP of 8 — marginal. Six sensors might give GDOP of 1.4 — excellent. By emitting greedily on 3 you are permanently throwing away the observations that would most improve your solution quality.
The correct design is to always wait for the full eviction window (50ms), collect all sensors that received the message, then emit. You use min_sensors only as a discard threshold at eviction time, not as an emit trigger.
WRONG (current): emit when n >= 3, discard sensors 4..N
CORRECT:         wait 50ms, emit if n >= 3, use all sensors that arrived
The 50ms wait is already accounted for in your pipeline timing. At 50Hz mock generation and 100ms Kalman update rate, a 50ms correlation window adds at most one update cycle of latency. For an aircraft moving at 250 m/s this is 12.5m of position drift — completely negligible.
The change to evict_stale is minimal: remove the early-emit path from ingest() entirely, and only emit from evict_stale(). The min_sensors check moves to eviction:

```
rustpub fn ingest(&mut self, frame: RawFrame) {
    // Accumulate only — never emit early
    let key = CorrelationKey::from_frame(&frame)?;
    let now_ns = frame.timestamp_ns();
    let group = self.groups.entry(key).or_insert_with(|| ...);
    if !group.frames.iter().any(|f| f.sensor_id == frame.sensor_id) {
        group.frames.push(frame);
    }
    // No emission here
}

// evict_stale emits when window expires — uses ALL collected sensors
pub fn evict_stale(&mut self, now_ns: u64) -> Vec<CorrelationGroup> {
    // emit groups older than window_ns that have >= min_sensors
    // this is where min_sensors is checked
}
```

This is the fundamental difference between a good MLAT system and a mediocre one. Every extra sensor you throw away is accuracy you're leaving on the table.

---

### 2. The Clock Sync Engine Calibrates Pairwise But MLAT Needs Per-Sensor Absolute Offsets

The current design stores offsets as `HashMap<(sensor_i, sensor_j), f64>` — the relative clock difference between each pair of sensors. This seems natural but creates a conceptual problem.

With N sensors you have N*(N-1)/2 pairs. With 8 sensors that's 28 pairs. But the underlying degrees of freedom in the clock problem is only N-1 — there are N clocks but you choose one as reference, leaving N-1 offsets to estimate. You're estimating 28 values when there are only 7 independent unknowns.

This over-parameterization causes two problems:

**Problem A: Inconsistency.** Your pairwise estimates may be mutually inconsistent. If offset(1→2) = 47ns and offset(1→3) = 31ns, then the implied offset(2→3) = -16ns. But your direct measurement of offset(2→3) might give -19ns due to noise. Now in `apply_corrections`, depending on which pair you look up, you get different results for the same TDOA. The solver sees residuals that cannot simultaneously be zero — the corrections themselves introduce noise.

**Problem B: Unobserved pairs.** If sensors 5 and 8 never both receive the same beacon aircraft message (possible if their coverage areas don't overlap), you never calibrate the pair (5,8) directly. You return `0.0` for unknown pairs, which means no correction — which is wrong if sensor 5 runs 200ns fast and sensor 8 runs 100ns slow.

**The correct approach:** estimate one absolute offset per sensor relative to a single reference sensor. Sensor 1 is reference (offset = 0 by definition). Estimate offset(2), offset(3), ..., offset(N). When correcting TDOA between sensors i and j:
```
corrected_tdoa(i→j) = raw_tdoa(i→j) - (offset_i - offset_j)
Now you have N-1 independent values, they're always consistent by construction, and an unobserved sensor pair is still handled — you just use their individual absolute offsets.
The estimation becomes: every beacon observation of sensors i and j gives you a measurement of (offset_i - offset_j). With a reference sensor fixed at 0, each pair observation constrains one unknown. Run a least-squares fit or simply accumulate all observations that involve sensor 1 to get absolute offsets directly.
```
3. The Windowed Median Breaks During Clock Drift
The median filter assumes the true clock offset is a constant. Real sensor clocks drift. A TCXO (temperature-compensated crystal oscillator) drifts at roughly 0.1-1 ppm. At 1 ppm, a 43,200-second-old midnight reference accumulates 43,200 * 1e-6 = 0.043 seconds = 43,000,000 nanoseconds of drift by noon. Even at 0.1 ppm that's 4,300ns — enough to cause meaningful MLAT error.
The windowed median with WINDOW_SIZE=50 and ~10 observations/second holds 5 seconds of data. Over 5 seconds a 1 ppm clock drifts 5 * 1e-6 * 1e9 = 5,000 ns — more than the window can track. The median of a window that spans a 5,000ns drift range will lag behind the true current offset by half the window's time span.
More subtly: the outlier gate uses 3 * spread.max(50.0). If the clock is actively drifting, the spread of the window will be large (because old observations and new observations genuinely differ), so the gate becomes very wide and passes everything, including actual multipath spikes. The outlier gate and drift tracking are in tension.
SO FINALLY KALMAN FILTER IS THE WAY TO GO HERE TOO.

4. The Spoof Detector Compares Haversine Distance Against a Fixed 2km Threshold — But Altitude Is Ignored
Location: spoof_detector.rs, SPOOF_THRESHOLD_M = 2000.0
rustlet p1 = point!(x: mlat_lon, y: mlat_lat);
let p2 = point!(x: claimed_lon, y: claimed_lat);
let divergence_m = p1.haversine_distance(&p2);
```

Haversine gives horizontal (2D surface) distance. MLAT gives 3D position. A spoofed aircraft that correctly reports its horizontal position but lies about its altitude by 3,000m would pass the spoof check entirely. Altitude spoofing is a real attack vector — an aircraft claiming to be at FL350 when MLAT puts it at ground level, or vice versa.

Also the 2km threshold is applied uniformly regardless of GDOP. If your MLAT solution has GDOP of 4.8 (just under your 5.0 discard threshold), horizontal accuracy might be 1,500m. A legitimate aircraft would be flagged as a spoofer even though the divergence is within MLAT's own uncertainty. The threshold should be:
```
flag if divergence_m > max(SPOOF_THRESHOLD_M, 3 * confidence_ellipse_m)
This way you only flag when the divergence genuinely exceeds what MLAT uncertainty could explain.


5. The MLAT Solver's Residual Function Has a Sign Convention That May Be Inverted
Location: MlatResiduals::cost()
rustlet residuals: Vec<f64> = self.observed_tdoa_m.iter().enumerate().map(|(k, obs)| {
    let dist_k = (p - &self.sensor_ecef[k + 1]).norm();
    obs - (dist_k - ref_dist)
}).collect();
```

The residual is `observed - theoretical`. The theoretical TDOA is `dist(aircraft, sensor[k+1]) - dist(aircraft, sensor[0])`. This is the distance to non-reference sensor minus distance to reference sensor.

But the observed TDOA is defined in `MlatInput` as `t_i - t_j` where index 0 is reference. So `observed_tdoa_m[k] = (t_{k+1} - t_0) * c`. If sensor k+1 is further from the aircraft, it receives the signal later, meaning `t_{k+1} > t_0`, meaning `observed_tdoa_m[k] > 0`. The theoretical TDOA for this case is `dist_{k+1} - dist_0 > 0`. Both positive. The sign convention is consistent.

However there's a subtler problem: the `MlatInput.tdoa_ns` is defined as `tdoa_ns[i][j] = t_i - t_j`. This is a 2D matrix. In the solver you seem to index it as a 1D vector of TDOAs relative to sensor 0. The data structure says it can hold all pairwise TDOAs but the solver only uses TDOAs relative to the reference sensor. The other N*(N-1)/2 - (N-1) independent pairwise TDOAs are wasted. With 6 sensors you have 15 pairwise TDOAs but only use 5. You're throwing away ~66% of your measurement information. This directly reduces overdetermination and worsens GDOP.

The correct approach is to use all independent pairwise TDOAs as separate equations in the residual function, giving you an overdetermined system with better noise rejection.

---

### 6. The Kalman Filter Velocity State Is Never Validated Against Physical Aircraft Limits

The Kalman state includes velocity `[vx, vy, vz]` in ECEF m/s. After initialization, the velocity starts at zero and is estimated from position updates. But position updates come from MLAT which has accuracy of ~100-500m. With 100ms update intervals, the Kalman filter will try to infer velocity from 100-500m noisy position differences — giving velocity estimates of `500m / 0.1s = 5000 m/s`. That's Mach 14.

The transition matrix propagates these absurd velocity estimates forward, predicting the next position will be `5000 m/s * 0.1s = 500m` away from the current position. Since MLAT fixes confirm the aircraft is still roughly where it was, the Kalman update fights the prediction constantly, never converging on sensible velocity.

There is no velocity observation in the plan — you only observe position. Velocity is inferred purely from position changes. With 100-500m MLAT noise this inference is extremely noisy.

Two fixes: either (a) add explicit velocity bounds — if predicted position change exceeds `max_speed * dt` (e.g. 300 m/s * 0.1s = 30m), clamp the prediction; or (b) dramatically increase process noise `sigma_a` so the filter doesn't trust its own velocity predictions, effectively reducing the filter to near-position-only. The current `sigma_a = 2 m/s^2` is too tight for a filter receiving 100-500m noisy position fixes at 10Hz.

---

### 7. The TDOA-to-Meters Conversion Has a One-Way Inconsistency

**Key implementation notes, line 2973:**
```
TDOA passed to solver: converted to meters (tdoa_ns * 0.299792458)
The value 0.299792458 is the speed of light in meters per nanosecond. So tdoa_ns * 0.299792458 = distance in meters. This is correct.
But in ClockSyncEngine:
rustconst C_M_PER_NS: f64 = 0.299_792_458;
let expected_tdoa_ns = (dist_i - dist_j) / C_M_PER_NS;
```

Here you divide distance by `c` to get nanoseconds. Also correct.

The inconsistency: `observed_tdoa_m` in `MlatResiduals` is TDOA in meters (multiplied by c). But `ClockSyncEngine` works in nanoseconds. When you apply clock corrections from `ClockSyncEngine` (which returns nanoseconds) to TDOAs before converting to meters for the solver, you must ensure the correction is applied before the multiplication, not after. If applied after:
```
corrected_tdoa_m = (raw_tdoa_ns * c) - correction_ns   // WRONG: unit mismatch
corrected_tdoa_m = (raw_tdoa_ns - correction_ns) * c   // CORRECT
The plan never explicitly shows where the correction is applied relative to the multiplication. Given that apply_corrections is currently a stub (Issue 3 from the implementation analysis), this unit ordering is unspecified and easy to get wrong when implementing.

8. The Mock Aircraft Use Linear Motion in Degree-Space, Not ECEF
Location: Mock aircraft definition and update loop
ruststart_lat: 51.0, vel_lat: 0.003,  // degrees/second
start_lon: 1.0,  vel_lon: 0.005,
Position updates: lat += vel_lat * dt; lon += vel_lon * dt
A degree of longitude at 51°N is 111,320 * cos(51°) = 70,000m, not 111,320m. An aircraft moving at vel_lat = 0.003 deg/s moves at 0.003 * 111,320 = 334 m/s northward. But vel_lon = 0.005 deg/s moves at 0.005 * 70,000 = 350 m/s eastward. These happen to be similar numbers so the distortion isn't catastrophic.
The real problem is that clock sync calibration uses this aircraft's position as ground truth. If the mock aircraft's position in ECEF (used for TDOA physics) differs from its claimed ADS-B position (what the clock sync engine receives), the calibration will measure a phantom clock offset. Specifically: TDOA physics is computed from ECEF positions (correctly, via wgs84_to_ecef), but the beacon position fed to update_from_beacon comes from... where exactly? The plan doesn't specify how the mock beacon aircraft's position is encoded in its mock ADS-B message. If it uses the degree-space position, and the TDOA physics uses the ECEF-converted position, and these are the same point expressed differently, there's no inconsistency. But if there's any coordinate conversion inconsistency in the mock data generation, clock sync will converge to wrong offsets that are baked into your test data, and you'll never know.

9. CPR Decoding Is Still a Placeholder — And There Is No Fallback That Actually Works
The plan still has:
rustNone // placeholder until CPR decoder implemented
And offers a "fallback" of using ADS-B velocity messages or trusting claimed positions for validated beacon aircraft. Neither is actually specified in working code.
This is a conceptual problem because the entire clock sync system is predicated on having decoded ADS-B positions. If CPR decoding isn't done, update_from_beacon is never called, get_offset always returns 0.0, and all MLAT runs on raw uncalibrated timestamps. For the mock mode this doesn't matter — you know the exact position of mock aircraft. But for live mode this means you're shipping a system where the key differentiating feature (clock calibration) silently does nothing.
The conceptual fix is to acknowledge this dependency explicitly in the phase structure: Phase 3 should begin with "if CPR decode is not complete, this entire phase produces no output — there is no partial functionality." Right now it's framed as if the rest of Phase 3 works regardless.

10. The Confidence Ellipse Formula Takes the Top-Left 2×2 of ECEF Covariance — This Is Not the Horizontal Covariance
This was mentioned briefly in the implementation analysis but deserves a fuller conceptual treatment.
The ECEF covariance matrix C has components [Cxx, Cxy, Cxz; Cyx, Cyy, Cyz; Czx, Czy, Czz]. The fixed_slice::<2,2>(0,0) takes [Cxx, Cxy; Cyx, Cyy] — the covariance in the ECEF X and Y directions.
At a latitude of 51°N (center of your sensor network), the ECEF X axis points roughly toward the prime meridian at the equator, and the ECEF Y axis points roughly toward 90°E at the equator. Neither of these aligns with "horizontal" as experienced by someone standing at 51°N. The true horizontal plane at 51°N is tilted relative to the ECEF XY plane by 51 degrees.
The result: your confidence ellipse radius is the uncertainty in a physically arbitrary plane, not the uncertainty a pilot or air traffic controller would recognize as "horizontal accuracy." Specifically, ECEF Z has a component in the local horizontal direction at mid-latitudes. By ignoring Z in your covariance slice, you're dropping some of the horizontal uncertainty.
The correct computation requires rotating the ECEF covariance into the local East-North-Up (ENU) frame and then taking the 2×2 horizontal (East-North) submatrix. The rotation uses a matrix derived from the observation point's latitude and longitude. For a hackathon this is arguably fine to skip, but the plan should document that confidence_ellipse_m is an approximation that underestimates horizontal uncertainty at high latitudes.


# Locus MLAT Implementation Plan — Theoretical & Conceptual Analysis

---

## Summary

The plan is ambitious and well-structured overall. However, it contains a number of theoretical errors and conceptual problems—some subtle, some severe—that would manifest as silent accuracy degradations, incorrect physics, or invalid ML results. Below is a component-by-component deep critique.

---

## 1. MLAT Solver (`mlat_solver.rs`)

### 1.1 Residual Sign Convention Is Physically Backwards

The residual in `MlatResiduals::cost()` is computed as:

```
residual[k] = observed_tdoa_m[k] - (dist(p, sensor[k+1]) - dist(p, sensor[0]))
```

However, the plan's TDOA convention (defined in the "Key Implementation Notes" section) is:

```
TDOA = t_sensor_i - t_sensor_ref
```

The theoretical TDOA for sensor `i` relative to reference sensor `0` is the **difference in propagation distances divided by c**:

```
TDOA_theoretical = (dist(aircraft, sensor_i) - dist(aircraft, sensor_0)) / c
```

So `theoretical_tdoa_m[k] = dist(p, sensor[k+1]) - dist(p, sensor[0])` is correct—but only if the sign of `observed_tdoa_m` was computed as `t_i - t_ref`, which means the signal arrived **later** at sensor `i` than the reference. This matches a positive range difference `dist_i > dist_ref`. So far so good.

The critical error is in the **mock TDOA generation** (Step 1.3):

```
arrival_time_ns = true_timestamp_ns + (dist / c_m_per_ns)
```

This defines `t_sensor = t_emission + dist/c`. Therefore:

```
TDOA_obs = t_i - t_ref = (dist_i - dist_ref) / c
```

So the sign is consistent—but the **units are wrong** in the solver. `tdoa_ns` (nanoseconds) is passed through the struct as `observed_tdoa_m`, implying it has already been converted to meters via `tdoa_ns * c_m_per_ns`. This conversion step is mentioned in the notes but **never appears in the solver's input construction code**. There is no explicit line in `main.rs` or anywhere that does this conversion before populating `MlatInput::tdoa_ns`. The field is named `tdoa_ns` but used as meters in the residual—a unit inconsistency that would cause the solver to converge to a position off by a factor of ~0.3 (i.e. `1/c` = ~3.34 ns/m), placing aircraft underground or outside Earth entirely.

### 1.2 GDOP Formula Is Wrong for MLAT

The GDOP computation builds an `N×3` geometry matrix of unit vectors from the solution to each sensor, then computes `sqrt(trace((H^T H)^{-1}))`.

This is the PDOP/GDOP formula for **GPS-style positioning**, where the receiver solves for 4 unknowns: `[x, y, z, clock_bias]`. The standard formula uses an `N×4` matrix with a `1.0` in the fourth column for the clock term.

MLAT using TDOA **already eliminates the clock bias** by differencing timestamps. The unknown space is only 3D: `[x, y, z]`. The correct TDOA geometry matrix is the **Jacobian of the residual vector**—the `N×3` matrix from Step 2.2 that is already computed for the LM solver. The correct GDOP is:

```
GDOP_MLAT = sqrt(trace((J^T J)^{-1}))
```

where `J` is the analytical Jacobian of the TDOA residuals, not of the unit-vector directions to sensors. Using the wrong H matrix produces GDOP values that are geometrically meaningless for MLAT and will not correlate with actual position uncertainty.

### 1.3 Initial Guess: Centroid Altitude Scaling Is Fragile

The fallback initial guess scales the sensor centroid outward by `(R_earth + 10000) / |centroid|`. This only works if all sensors are at roughly the same altitude, which is not true in general—Dublin (20m), Zurich (440m), and Vienna (170m) differ by hundreds of meters, but more importantly, the ECEF centroid is already ~6.37 million meters from Earth's center so the scaling is fine numerically. However, the **direction** of the centroid is wrong for aircraft significantly off-axis from the sensor network. For example, if the aircraft is at lat=51, lon=1 (English Channel) and the sensors span London to Vienna (lon ~0 to 16), the centroid is at ~lon 6, pushing the initial guess ~5 degrees away from the true position. For a network spanning 2000km, a 5-degree error is ~550km—well outside the convergence basin of LM without a good Jacobian. The plan acknowledges LM is sensitive to initialization, but the proposed fallback may not be sufficient to guarantee convergence without the Kalman prior, which itself is unavailable on first observation.

### 1.4 `argmin` LevenbergMarquardt vs GaussNewton—API Conflict

The import comment says:
```rust
use argmin::solver::leastsquares::GaussNewton;
// or: argmin::solver::levenbergmarquardt::LevenbergMarquardt
```

In `argmin 0.9`, `LevenbergMarquardt` requires implementing `Jacobian` in addition to `CostFunction`. The plan does describe the Jacobian, but uses a `DVector<f64>` output type for the residuals. `argmin`'s LM solver actually expects `residuals` as a type that implements `ArgminL2Norm`—the exact API depends on version. The plan lists `argmin = "0.9"` and `argmin-math = "0.4"`, but there is no verification that the `nalgebra::DVector` return type from `CostFunction` is compatible with `argmin 0.9`'s LM trait bounds. This is a latent compile-time error risk, not a theoretical problem per se, but it reflects a gap in plan validation.

---

## 2. Clock Synchronization (`clock_sync.rs`)

### 2.1 Fundamental Circularity: Clock Sync Depends on Known Aircraft Position

The `update_from_beacon` function computes the expected TDOA using `beacon_ecef`—the known position of a beacon aircraft. But in the live system, the beacon's position is itself decoded from CPR-encoded ADS-B messages, and CPR decoding is explicitly left as a `TODO` returning `None` (Step 3.1). 

This creates a **hard bootstrap dependency**: clock sync cannot work without beacon position; the solver produces poor results without clock sync; and beacon position cannot be obtained without CPR decoding. The plan acknowledges this but proposes using the "ADS-B velocity message (TC=19) combined with known reference points" as a fallback, which is not a valid substitute—velocity doesn't give absolute position. The only valid bootstrap path is: decode CPR first, use a subset of geometrically favorable, unclocked sensor pairs (where timing errors from clock drift are tolerable at short baseline), and then iteratively improve.

### 2.2 Pairwise Offsets Don't Form a Consistent System

The engine stores and applies pairwise offsets `(sensor_i, sensor_j)` independently. However, clock offsets must satisfy transitivity: if sensor A is +5ns ahead of B, and B is +3ns ahead of C, then A must be +8ns ahead of C. The plan makes no attempt to enforce this constraint. With `N=8` sensors, there are `N(N-1)/2 = 28` sensor pairs, but only `N-1 = 7` independent offsets. Storing and applying 28 independent medians leads to an **over-parameterized and potentially inconsistent** correction system. When corrections from different pairs conflict, the residuals in the solver won't reduce to zero even with a perfect aircraft position—the solver will converge to a biased solution.

The correct approach is to estimate `N-1` absolute clock offsets (relative to one reference sensor) in a least-squares sense across all beacon observations, not maintain pairwise medians independently.

### 2.3 `apply_corrections` Has No Effect

The implementation of `apply_corrections` is essentially a no-op:
```rust
let correction = self.get_offset(frame.sensor_id, ref_id);
let _ = correction; // applied at TDOA computation site
```

The correction is computed and then discarded with `let _ = correction`. The comment says it's "applied at TDOA computation site," but there is no corresponding implementation at any TDOA computation site in the plan. This means clock corrections are computed and logged but **never actually applied to TDOA values** passed to the solver.

### 2.4 Outlier Gate References Undefined Spread When Buffer is Small

The outlier rejection gate activates when `buf.len() >= 5`, computing the current spread. However, `spread_of_deque` computes the Median Absolute Deviation (MAD). When `buf.len()` is exactly 5, the MAD is computed from 5 values of `(x - median).abs()`. With 5 samples, 3 of which may cluster near one value and 2 near another, the MAD can be 0 if ≥3 values are identical (which can happen with quantized nanosecond timestamps). The `3.0 * current_spread.max(50.0)` floor of 50ns prevents this degenerate case—but 50ns is a very loose gate: at 300 m/μs, 50ns ≈ 15m of range error, which is actually fine for MLAT. However, once the window fills and the true spread is <16ns (typical for stable receivers), the gate tightens to 48ns, potentially rejecting valid observations from aircraft at high elevation angles where timing variability is legitimately larger.

---

## 3. Correlator (`correlator.rs`)

### 3.1 Content Hash Doesn't Handle Bit Errors

The correlation key uses `FxHash(raw_bytes)`. In real Mode-S reception, the same aircraft transmission received by two different sensors will sometimes have **single-bit errors** in one reception (before CRC correction), producing different hashes for the same logical message. This is particularly common at low SNR or at sensor range extremes. The standard MLAT correlator approach uses the ICAO24 address plus a 24-bit CRC check—frames that pass CRC on any sensor are considered the same message. The plan's hash-based approach will silently fail to correlate frames with marginal reception quality, reducing the number of usable MLAT fixes for distant or attenuated aircraft.

### 3.2 Correlation Window of 50ms Is Too Wide for Live Data, Too Narrow for Mock

The `window_ns = 50_000_000` (50ms) is described as "larger than max EU propagation spread of 6ms." While 6ms is the max one-way propagation across a ~1800km European network (6ms ≈ 1800km/c), the **actual reception time spread** depends on the aircraft's position relative to sensors. For an aircraft equidistant from all sensors (near the network centroid), TDOA is ~0. For an aircraft near one edge, TDOA could be up to 6ms. However, 50ms is 8× this maximum and will cause the correlator to **hold groups open for 50ms** even when only 2 sensors have seen the frame—accumulating frames from **multiple aircraft transmissions** of the same message type in rapid succession. At ADS-B message rates of 2 Hz per aircraft per message type, 50ms is well below the inter-message interval, so this is probably acceptable. But for the mock mode generating at 50Hz per aircraft, 50ms windows will accumulate 2-3 mock "transmissions" of the same aircraft.

### 3.3 Immediate Emission at `min_sensors` May Discard Better Geometry

When `min_sensors=3` are reached, the group is **immediately emitted and removed**. If sensors 4-8 would have also received the message (within the 50ms window), their timing data is lost—the group has been consumed. This means the solver always gets exactly 3 observations when 3+ sensors are in range, instead of the potentially 7-8 that could be collected. More observations = better GDOP = better accuracy. The correct design is to emit at window expiry (collecting all sensors) and use `min_sensors` only as a minimum threshold at that point—not as an early-exit trigger.

---

## 4. Kalman Filter (`kalman.rs`)

### 4.1 Process Noise Matrix Q Is Incomplete

The `process_noise(dt)` function builds a matrix with the diagonal blocks for position and velocity, and the off-diagonal coupling terms `(0,3)`, `(1,4)`, `(2,5)`. However, a correct constant-acceleration/constant-velocity process noise uses the full Singer model or Wiener process model. The formula given has:
- `q[(0,0)] = dt^3/3 * σ_a²` ✓  
- `q[(3,3)] = dt * σ_a²` ✓  
- `q[(0,3)] = dt^2/2 * σ_a²` ✓

But the code uses `dt2 = dt*dt/2.0` and `dt3 = dt*dt*dt/3.0`, then assigns `q[(0,0)] = dt3 * sigma_a2`. This is correct. However, the code is **missing the cross-coupling between y–vy and z–vz** in the off-diagonal terms—only (0,3), (1,4), (2,5) are set. ECEF components are independent (which is correct), so this is fine. But the missing concern is that in ECEF coordinates, the **process noise should not be the same in all three dimensions**: the vertical uncertainty (radial) is typically much smaller for a cruising aircraft than horizontal. Using the same `σ_a = 2 m/s²` for x, y, and z in ECEF is physically wrong—horizontal ECEF motion maps to a combination of lateral and along-track directions, but vertical motion in ECEF is dominated by altitude changes. A cruising aircraft at 10,000m barely changes altitude (σ_a_vertical << 2 m/s²), so the Kalman will be over-confident in horizontal predictions and under-confident in vertical ones. This won't cause divergence, but it will degrade the filter.

### 4.2 EKF Is Not an EKF

The plan calls this an Extended Kalman Filter (EKF), but the implementation is a **standard linear Kalman Filter**. The observation model `H` is the identity `[I | 0]` mapping ECEF position directly—this is linear, not nonlinear. An EKF is only needed when the measurement model involves a nonlinear transformation (e.g., measuring range, angle, or spherical coordinates). Since MLAT output is already converted to ECEF before being fed to the filter, linearization isn't required, and the filter is just a KF. This is actually correct behavior, but it's a mislabeling that could cause confusion or lead someone to incorrectly add Jacobian linearization steps that aren't needed.

### 4.3 Kalman Update's `try_inverse` Fallback to Zero Matrix Is Dangerous

```rust
let k = &self.p * h.transpose() *
    s.try_inverse().unwrap_or(nalgebra::Matrix3::zeros());
```

If `S = H P H^T + R` is singular (which shouldn't happen for a well-formed R, but could occur due to numerical issues), the Kalman gain `K` becomes the zero matrix, causing the update step to be completely skipped. This is silently wrong—the filter will not diverge, but it will stop incorporating measurements, potentially indefinitely, without any log warning. A singular `S` should be treated as an error (or at minimum a warning), not silently neutralized.

### 4.4 Vertical Noise Factor Is Inconsistently Justified

In `update()`:
```rust
&nalgebra::Vector3::new(r_val, r_val, r_val * 4.0)
// Vertical (z) is ~2x less accurate in MLAT geometry
```

The comment says "~2x less accurate" but the code uses `4.0× r_val`, which is the **variance** being 4×, corresponding to a **standard deviation** of 2×. The comment is ambiguous (2× std or 2× variance?) but the code choice of factor 4 for the variance is the geometrically consistent one. However, "less accurate in z" is only true in a **local ENU frame**—not in ECEF. In ECEF, the "z" component (roughly northward at European latitudes, with some upward tilt) does not directly correspond to the geometric vertical. The vertical accuracy degradation in MLAT comes from poor vertical DOP (the aircraft is always above the sensors), which manifests in the **local Up direction**, not ECEF z. The R matrix anisotropy should be applied after rotating the ECEF covariance into ENU, not directly on z.

---

## 5. CPR Decoder — Fundamental Theoretical Gap

### 5.1 The `parse_adsb_position` Function Always Returns `None`

Step 3.1 defines the CPR decoder but explicitly returns `None` as a placeholder. This is called out as a `TODO`, but the plan's fallback suggestions are theoretically invalid:

> "use the aircraft's self-reported position via the ADS-B velocity message (TC=19) combined with known reference points"

TC=19 (airborne velocity) messages contain groundspeed and heading—they do not contain position. There is no way to derive absolute position from velocity without integration from a known starting point, which requires a separate position fix.

The alternative of "trusting claimed lat/lon only for beacon aircraft validated by two ground stations" conflates ADS-B spoofing detection (which hasn't been set up yet) with clock sync bootstrapping, creating a circular dependency.

**Without CPR decoding, the entire clock sync system produces no output**, meaning Phase 3 is completely non-functional in its primary goal.

### 5.2 CPR Global Decode Requires Even/Odd Frame Pairs from Same Aircraft Within 10 Seconds

The `CprDecoder` struct stores `even` and `odd` frame pairs per aircraft, which is correct. However, the plan omits the ICAO Doc 9684 constraint that the two frames must arrive within **10 seconds** of each other, or the global decode may place the aircraft in the wrong CPR zone (a ~360nm disambiguation error). Additionally, the standard global decode produces two candidate positions—one of which is rejected by being outside a reasonable geographic bound. The plan makes no mention of either of these constraints.

---

## 6. Spoof Detector (`spoof_detector.rs`)

### 6.1 Haversine Distance Used for ADS-B vs MLAT Comparison—Altitude Ignored

```rust
let divergence_m = p1.haversine_distance(&p2);
```

Haversine computes **horizontal great-circle distance** only, ignoring altitude. For spoof detection comparing `(lat, lon, alt_m)` positions, ignoring altitude is incorrect: a spoofer could broadcast a false altitude while keeping lat/lon correct, or vice versa. The full 3D distance should use the Euclidean distance in ECEF space, which already encodes altitude correctly.

### 6.2 No Temporal Coherence — Instantaneous Comparison Only

The spoof detector compares a single MLAT fix against the most recent stored ADS-B claim. This has two problems:

1. **ADS-B update rate vs MLAT rate**: ADS-B position messages broadcast at ~2 Hz; MLAT solves may come at 10+ Hz. The "most recent" ADS-B claim may be 500ms old and the aircraft has moved, producing a false positive divergence for fast-moving aircraft.

2. **No persistence**: A single-frame divergence sets `spoof_flag=true`, but there's no minimum persistence requirement (e.g., "divergence must exceed threshold for N consecutive frames"). A single noisy MLAT solution or multipath-corrupted ADS-B frame will trigger false alarms.

The threshold of 2km without temporal smoothing will produce false positives at low GDOP conditions where MLAT positional uncertainty itself may be >2km.

### 6.3 The Spoofing Threat Model Is Too Simple

The plan models spoofing as ADS-B position divergence from MLAT truth. But the more dangerous threat for which MLAT is used in practice is **the aircraft broadcasting a false ICAO24 address** (identity spoofing), not just false position. The system has no mechanism to detect this case—an aircraft transmitting as `AABBCC` but physically located elsewhere will produce the right MLAT position for the physical aircraft's true location, but the ICAO24-keyed ADS-B record might belong to a completely different aircraft's position.

---

## 7. ML Service (`ml-service/`)

### 7.1 Training on Raw Lat/Lon Is Fundamentally Scale-Inconsistent

The LSTM input features are `[lat, lon, alt, vel_lat, vel_lon, vel_alt]`. Latitude and longitude in degrees have a range of roughly 45–55° and -5–15° respectively (European airspace), while altitude ranges 5,000–12,000m and velocity in degrees/second is ~0.003–0.008. These features differ in magnitude by 6 orders of scale and have **completely different physical dimensions**. Although z-score normalization is applied, the normalization is computed over the training distribution—if the operational distribution covers different geographic areas, the normalization fails.

More critically, **longitude degrees are not commensurate with latitude degrees** (1° longitude at 51°N ≈ 63km vs 1° latitude ≈ 111km). Using raw degrees makes the model sensitive to the aircraft's east-west vs. north-south trajectory directionality in a physically meaningless way. The correct approach is to work in ECEF meters or local ENU meters, where spatial dimensions are commensurate.

### 7.2 Synthetic Training Data Doesn't Match the Input Distribution

The mock MLAT system produces Kalman-smoothed ECEF positions converted to WGS84. The training data is generated as:
```python
lat += vl * 0.1; lon += vlo * 0.1; alt += va * 0.1
```

This is raw (lat, lon, alt) integration—not Kalman-filtered, not ECEF-based, not subject to measurement noise. The synthetic training distribution therefore does not match the operational input distribution the model will actually see. The LSTM will be trained on clean, noise-free, analytically generated tracks and will encounter noisy, Kalman-smoothed tracks at inference time.

### 7.3 Anomaly Label Definition Is Circular with Spoof Detection

The anomaly classifier's training data defines "anomalous" as a position jump:
```python
track[jump_idx:, 0] += np.random.uniform(0.02, 0.08)  # lat jump (spoof)
```

But the spoof detector in `spoof_detector.rs` already catches exactly this behavior—ADS-B position divergence from MLAT. If the MLAT position is accurate, the spoof detector will flag spoofed aircraft before the ML classifier ever sees the track. The ML classifier is therefore learning to detect the same thing as the rule-based spoof detector, providing no additional information.

For the ML classifier to be genuinely useful, it should target **behaviors the rule-based system cannot detect**: unusual flight paths (e.g., low-altitude overflights, erratic maneuvering, unexpected airspace intrusions), or **classification of intent** beyond mere spoofing. Training on position jumps alone makes it a weaker duplicate of the spoof detector.

### 7.4 80/20 Train/Val Split Without Shuffling by Aircraft Identity

```python
split = int(0.8 * len(X_norm))
X_train, X_val = X_norm[:split], X_norm[split:]
```

The dataset is constructed as 800 normal + 200 anomalous tracks, appended in order. An 80/20 split gives the first 800 samples to train and the last 200 to validation. Since the anomalous samples are appended **at the end**, approximately **all 200 anomalous samples end up in the last 200 entries**. The validation set therefore contains **only anomalous samples**, making the validation accuracy meaningless (it measures only anomalous recall, not balanced accuracy). This also means the training set sees 800 normal and 0 anomalous samples—the model won't learn to detect anomalies at all.

The code does shuffle the combined array before splitting (`idx = np.random.permutation`), which resolves this. But if the shuffle is removed during future debugging, the split becomes catastrophically wrong. The proper approach is to shuffle before appending or use sklearn's `train_test_split` with stratification.

### 7.5 Zero-Padding of Short Sequences Is Semantically Wrong

```python
if len(req.states) < SEQ_LEN:
    pad = SEQ_LEN - len(req.states)
    seq = [[0.0] * FEATURES] * pad + [...]
```

Padding with zeros at the start means the LSTM sees `[0, 0, 0, 0, 0, 0]` as if the aircraft was at lat=0, lon=0, alt=0, velocity=0 for the first N frames—which is in the middle of the Atlantic Ocean at sea level. Since the model was trained on European airspace data (lat 48–54, lon -2 to 12), a zero-padded sequence will always start with a distribution shift, likely causing the LSTM to misclassify early-track data. The correct approach is to either pad with the first valid observation repeated, or mask padded timesteps using a sequence mask.

### 7.6 Global Variable Model State and Thread Safety

The FastAPI service uses global `model`, `X_mean`, `X_std` variables:
```python
model: Optional[AircraftLSTM] = None
X_mean: Optional[np.ndarray] = None
```

The `/train` endpoint's background task calls `load_model()` which reassigns these globals mid-inference. With Uvicorn's async concurrency, a `/classify` request could read `model` between `model = m` and the corresponding `X_mean`/`X_std` updates, inferring with the new model weights but the old normalization parameters. This is a data race on the model state, which would produce incorrect classifications without any error.

---

## 8. Timestamp Representation

### 8.1 "Seconds Since Midnight" Breaks at Midnight and Across Days

The `RawFrame` timestamp is stored as:
- `timestamp_seconds: u64` — seconds since midnight UTC
- `timestamp_nanoseconds: u64` — nanosecond component within that second

Reconstituted as: `timestamp_ns = timestamp_seconds * 1e9 + timestamp_nanoseconds`

This design has a fundamental ambiguity: it gives no indication of **which day**. At midnight UTC, `timestamp_seconds` wraps to 0. If frames from two different sensors span midnight (one at `23:59:59.999` and another at `00:00:00.001`), the TDOA computed from their timestamps will be ~86,400 seconds ≈ 86.4 trillion nanoseconds instead of ~2ms. This would corrupt any correlation and TDOA computation that spans the midnight boundary.

The plan also doesn't address the case where the 4DSky SDK's `timestamp_seconds` is already relative to a different epoch (Unix time, GPS time, etc.)—this is an unresolved integration assumption.

### 8.2 Nanosecond Precision in `f64` Loses Significance

Clock offsets are stored as `f64` in nanoseconds. Absolute timestamps are up to `86400 * 1e9 = 8.64e13` nanoseconds. A `f64` has 53 bits of mantissa, giving ~15 significant decimal digits. At magnitude 8.64e13, precision is `8.64e13 / 1e15 ≈ 0.086` ns—so about 0.1ns resolution, which is sufficient. However, the mock sensors apply `Gaussian(σ=50ns)` noise, and the expected TDOA corrections are sub-10ns. This is fine, but at the boundary of practical floating-point significance for this representation. A safer representation is to store timestamps as `i64` nanoseconds relative to a fixed epoch, which is the standard in precision timing.

---

## 9. Dependency Graph / Architecture

### 9.1 ML Service Is Synchronous in the Inference Hot Path

In `anomaly_client.rs`, the Rust backend calls `POST /classify` for every aircraft with a new Kalman update. With 3 aircraft at 10 Hz MLAT rate and an ML inference latency of ~10ms per call (realistic for CPU PyTorch), this generates 30 HTTP requests/second, each blocking a Tokio task for 10ms. Tokio's async HTTP client (`reqwest`) handles this, but if the ML service is slow (model loaded on CPU, busy with training), calls will queue. The plan has no timeout handling in `anomaly_client.rs`—a slow ML service will backpressure the entire Kalman update pipeline.

### 9.2 WebSocket Broadcast Channel Capacity Is Undersized

The broadcast channel is created with capacity 1024. At 3 aircraft × 10 Hz = 30 updates/second, plus sensor health messages every 5s, the channel is fine for a handful of clients. However, `tokio::sync::broadcast` drops messages to **lagged receivers** (i.e., slow WebSocket clients), which is handled with a warning log but not surfaced to the frontend. A slow browser client will silently miss aircraft state updates.

---

## 10. Minor Structural Issues

### 10.1 `CorrelationGroup` Is Defined Twice

In Step 2.1, `CorrelationGroup` is defined without `first_seen_ns`, then immediately re-defined with it (lines 1104–1119). This appears to be an editing artifact in the plan, but if copied directly, it will cause a compile error (`duplicate type definition`).

### 10.2 Docker Compose: `go-ingestor` Has No Health Check

The `docker-compose.live.yml` override uses `condition: service_started` instead of `service_healthy` for `go-ingestor`, explicitly noting there is no health endpoint. The Go ingestor is the data source for the entire pipeline—if it crashes silently after starting (e.g., bad credentials, libp2p connection failure), Rust will retry indefinitely but the system will appear running while producing no data. A simple health check (e.g., `nc -z localhost 61336`) would help.

### 10.3 `sensor_j` vs `sensor_i` Index Convention in `apply_corrections`

```rust
let correction = self.get_offset(frame.sensor_id, ref_id);
```

But `get_offset` is keyed on `(sensor_i, sensor_j)` where observations are stored under `(sensor_i, sensor_j)` from `update_from_beacon`. If `update_from_beacon` was always called with `(i, j)` in a consistent order (e.g., lower ID first), then looking up `(frame.sensor_id, ref_id)` may miss entries stored as `(ref_id, frame.sensor_id)`. The `HashMap` lookup is order-sensitive. This would cause `get_offset` to always return 0.0 for pairs whose order was inverted during insertion.

### 10.4 Committed Model File Is Not Reproducible

The `.gitignore` explicitly states:
```
# NOTE: ml-service/*.pt is NOT gitignored — commit the trained model for reproducibility
```

But the model is trained on randomly generated synthetic data with no fixed random seed anywhere in `train.py`. Two runs of `train.py` will produce different weights, making the committed `.pt` file non-reproducible. For reproducibility, `torch.manual_seed(42)` and `np.random.seed(42)` should be set before generating data and initializing the model.

---
