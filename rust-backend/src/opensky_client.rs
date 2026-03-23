//! On-demand OpenSky REST API client.
//! Fetches callsign/squawk/country for a single aircraft on request,
//! with a 5-minute per-ICAO24 TTL cache to avoid repeat calls.

use std::{collections::HashMap, sync::Arc, time::{Duration, Instant}};

use tokio::sync::RwLock;

pub const CACHE_TTL: Duration = Duration::from_secs(crate::consts::OPENSKY_CACHE_TTL_SECS);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(crate::consts::OPENSKY_REQUEST_TIMEOUT_SECS);

/// Enrichment fields from OpenSky for one aircraft.
#[derive(Debug, Clone, Default)]
pub struct OpenSkyInfo {
    pub callsign:       Option<String>,
    pub squawk:         Option<String>,
    pub origin_country: Option<String>,
}

/// Cache entry with TTL timestamp.
pub struct CachedInfo {
    pub info:       OpenSkyInfo,
    pub fetched_at: Instant,
}

pub type OpenSkyCache = Arc<RwLock<HashMap<String, CachedInfo>>>;

pub fn new_cache() -> OpenSkyCache {
    Arc::new(RwLock::new(HashMap::new()))
}

pub fn new_client() -> reqwest::Client {
    reqwest::Client::builder()
        .timeout(REQUEST_TIMEOUT)
        .build()
        .expect("failed to build OpenSky HTTP client")
}

/// Fetch OpenSky state vector for a single ICAO24.
/// Returns `None` if the aircraft is not found, the API errors, or rate-limits.
/// The caller must check the TTL cache before calling this.
pub async fn fetch_aircraft_info(
    client: &reqwest::Client,
    icao24: &str,
) -> Option<OpenSkyInfo> {
    let url = format!(
        "https://opensky-network.org/api/states/all?icao24={icao24}"
    );

    let resp = match client.get(&url).send().await {
        Ok(r) => r,
        Err(e) => {
            tracing::warn!("OpenSky: request error for {icao24}: {e}");
            return None;
        }
    };

    if !resp.status().is_success() {
        tracing::warn!("OpenSky: HTTP {} for icao24={icao24}", resp.status());
        return None;
    }

    let body: serde_json::Value = match resp.json().await {
        Ok(v) => v,
        Err(e) => {
            tracing::warn!("OpenSky: JSON parse error for {icao24}: {e}");
            return None;
        }
    };

    // states is null when aircraft is not in coverage
    let sv = body.get("states")?.as_array()?.first()?;

    // OpenSky state vector field indices:
    // 0=icao24, 1=callsign, 2=origin_country, 14=squawk
    let callsign = sv.get(1).and_then(|v| v.as_str())
        .map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
    let origin_country = sv.get(2).and_then(|v| v.as_str())
        .map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
    let squawk = sv.get(14).and_then(|v| v.as_str())
        .map(|s| s.trim().to_string()).filter(|s| !s.is_empty());

    tracing::debug!("OpenSky: fetched {icao24} → callsign={callsign:?} squawk={squawk:?}");
    Some(OpenSkyInfo { callsign, squawk, origin_country })
}
