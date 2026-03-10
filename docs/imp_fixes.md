Unresolved Bugs & Mistakes
🔴 Critical
1. ${MOCK_DATA:+--mock} crashes container on startup
Docker Compose doesn't support ${VAR:+word} shell expansion. It will pass the literal string to the binary; clap rejects it as an unknown argument. Since MOCK_DATA is already wired via env = "MOCK_DATA" in clap, just delete the shell expansion entirely — the env var alone is sufficient.
2. Sensor health chart always shows undefined-undefined
The frontend accesses o.sensor_i / o.sensor_j, but Rust's serde serialises sensor_pair: (i64, i64) as a JSON array [1001, 1002]. Fix: either split the Rust struct into two named fields (sensor_i, sensor_j), or access them in JS as o.sensor_pair[0] / o.sensor_pair[1].
3. Clock corrections are never applied
apply_corrections() still contains let _ = correction. The comment documents the correct pattern but no code in the plan actually performs corrected_tdoa_ns = raw_tdoa_ns - get_offset(...) before multiplying by c. The TDOA assembly call site doesn't exist anywhere in the document.
🟠 High
4. GDOP formula is wrong for MLAT
compute_gdop() uses GPS-style direction cosine rows (unit vectors from solution to each sensor). The correct MLAT GDOP uses the TDOA Jacobian rows, which are already computed analytically for the LM solver: J[k][i] = (p_i - s_{k+1,i})/d_{k+1} - (p_i - s_{0,i})/d_0. Reuse that matrix instead of building a separate one.
5. Pairwise clock offsets are an inconsistent over-parameterised system
With 8 sensors there are 28 stored pairwise medians but only 7 independent offsets. There's no transitivity enforcement, so corrections from different pairs will conflict. The solver residuals won't reduce to zero even with a perfect aircraft position. Fix: solve for N−1 absolute offsets (relative to one reference sensor) in a least-squares sense across all beacon observations.
6. Covariance matrix is never computed — confidence_ellipse_m call site is incomplete
The function confidence_ellipse_m(&covariance) is called, but the plan never shows code that produces covariance from the argmin result. argmin's Executor result state does not expose (J^T J)^{-1} automatically; it must be computed explicitly at the solution point.
7. now_ns clock source for correlator eviction is unspecified
The mock generator uses simulated timestamps (large Unix-epoch nanosecond values). If evict_stale(now_ns) is fed wall-clock nanoseconds, groups are evicted instantly (wall >> simulated). If fed the last frame's timestamp, it works for mock but may lag for sparse live data. The plan never shows where now_ns comes from in the main loop.
8. libp2p peer ID → raw public key extraction is deferred and likely wrong
The location-override lookup uses a 33-byte compressed secp256k1 public key as the key. But stream.Conn().RemotePeer() returns a peer.ID, which is a multihash of the public key — not the raw key bytes. If the SDK uses Ed25519 (32 bytes) or the multihash wrapper is not stripped correctly, no override will ever match. This needs to be resolved before the hackathon, not at live-test time.
9. RawFrame::timestamp_ns() doc comment still says "since midnight"
The field is now Unix epoch seconds, so the returned value is ~1.7 × 10¹⁸ ns. Code or humans treating it as a time-of-day value will be silently wrong. Rename the method or fix the comment.
🟡 Medium
10. CPR decoder: no staleness check on even/odd frame pairs
If an aircraft disappears and returns, the stale stored half-pair combines with a new one from a different position. ICAO Doc 9684 requires the pair to arrive within 10 seconds. Store a timestamp with each half-pair and discard if age difference > 10 s.
11. CPR latitude normalisation misses values in [90°, 270°)
The formula only subtracts 360 when lat >= 270. Values in [90, 270) from the modular arithmetic represent invalid results (southern hemisphere decoded incorrectly). The correct check is: if lat >= 270, subtract 360; if lat > 90 && lat < 270, the decode is likely in the wrong zone.
12. Kalman R matrix vertical anisotropy applied in ECEF-Z, not local Up
r_val * 4.0 on the ECEF-z component is geometrically wrong. MLAT vertical degradation is in the local ENU Up direction, which at ~51°N maps to a combination of ECEF-x, y, and z. The R anisotropy should be applied after rotating into ENU. In practice this degrades altitude estimates by ~20–40%.
13. GDOP-adaptive spoof threshold can be exploited
threshold = max(2000m, 3 * accuracy_m). A spoofer that manoeuvres to be near-collinear with sensors degrades MLAT accuracy, raises the threshold, and makes itself harder to detect. This is a known limitation worth documenting.
14. First-state padding in ML inference masks early anomalies
Padding short sequences by repeating the first state tells the LSTM the aircraft was stationary at its entry point for several seconds, artificially reducing anomaly scores during the first ~2 seconds of track. Use a PackedSequence mask or accept this as a demo limitation.
🟢 Low / Design Gaps
15. ML features use incommensurate degree scales
[lat°, lon°, alt_m, vel_lat °/s, vel_lon °/s] — 1° longitude at 51°N ≈ 63 km vs 1° latitude ≈ 111 km. The model learns spurious directional biases. Use local ENU metres or at least multiply vel_lon by cos(lat) before feeding the model.
16. Synthetic training distribution doesn't match operational data
Training tracks are clean analytical integration; inference data is Kalman-smoothed with 100–500m MLAT noise. The LSTM will encounter a distribution it was never trained on.
17. ML classifier is a weaker duplicate of the spoof detector
Anomaly class is defined as a position jump, which the rule-based spoof detector already catches. The LSTM adds no new detection signal. Train it on manoeuvre-based anomalies (unusual climb rates, airspace violations, heading reversals) instead.
18. vel_lon division by zero near poles
east / (111_320.0 * lat_r.cos()) — no guard for cos(lat) ≈ 0. Irrelevant for European airspace, but produces inf → NaN in the LSTM if it ever fires.

➕ New Feature: Sensor Geometry / Collinearity Filter
The current plan has no pre-solve geometry check — it relies on GDOP > 5 post-hoc to discard bad solutions. This is wasteful (the solver runs to completion on degenerate geometry) and doesn't distinguish why geometry is bad.
Add a check_geometry() function in mlat_solver.rs called before the LM solver:
rust/// Returns None if sensor geometry is too poor to solve.
/// Checks three conditions:
///   1. Minimum sensor spread (baseline) — rejects clustered sensors
///   2. Collinearity — rejects sensors that are nearly coplanar in 3D
///   3. Pre-estimated GDOP from sensor positions alone (no solution needed)
pub fn check_geometry(
    sensor_ecef: &[Vector3<f64>],
    min_baseline_m: f64,       // e.g. 50_000.0  (50 km)
    max_condition_number: f64, // e.g. 1e6
) -> Result<(), &'static str> {

    // 1. Minimum baseline: at least one sensor pair must be > min_baseline
    let max_dist = sensor_ecef.iter()
        .flat_map(|a| sensor_ecef.iter().map(move |b| (a - b).norm()))
        .fold(0.0_f64, f64::max);
    if max_dist < min_baseline_m {
        return Err("sensors too clustered — baseline below minimum");
    }

    // 2. Collinearity / coplanarity: build centred matrix, check singular values
    let centroid = sensor_ecef.iter().fold(Vector3::zeros(), |a, s| a + s)
                   / sensor_ecef.len() as f64;
    let mut mat = nalgebra::DMatrix::<f64>::zeros(sensor_ecef.len(), 3);
    for (i, s) in sensor_ecef.iter().enumerate() {
        let d = s - &centroid;
        mat[(i,0)] = d[0]; mat[(i,1)] = d[1]; mat[(i,2)] = d[2];
    }
    let svd = mat.svd(false, false);
    let sv = svd.singular_values;
    // If smallest singular value near zero → sensors are coplanar/collinear
    if sv[sv.len()-1] < 1.0 {
        return Err("sensors coplanar/collinear — TDOA system rank-deficient");
    }
    // Condition number of sensor geometry matrix
    let condition = sv[0] / sv[sv.len()-1];
    if condition > max_condition_number {
        return Err("sensor geometry ill-conditioned");
    }

    // 3. Pre-solve GDOP estimate using sensor centroid as proxy solution
    // (saves running LM on geometry that will exceed GDOP threshold anyway)
    let proxy = centroid.normalize()
                * (6_371_000.0 + 10_000.0); // centroid direction + cruise altitude
    let gdop = compute_gdop(&proxy, sensor_ecef);
    if gdop > 5.0 {
        return Err("pre-solve GDOP estimate exceeds threshold");
    }

    Ok(())
}
Wire into solve():
rustpub fn solve(input: &MlatInput, prior: Option<Vector3<f64>>) -> Option<MlatSolution> {
    let sensor_vecs: Vec<Vector3<f64>> = input.sensor_positions.iter()
        .map(|&(x,y,z)| Vector3::new(x,y,z))
        .collect();

    if let Err(reason) = check_geometry(&sensor_vecs, 50_000.0, 1e6) {
        tracing::debug!("geometry rejected: {reason}");
        return None;
    }
    // ... rest of solve
}
