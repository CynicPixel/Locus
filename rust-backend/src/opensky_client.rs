//! Periodic OpenSky REST API poller.
//! Fetches live state vectors every POLL_INTERVAL and stores callsign/squawk
//! keyed by ICAO24 (lower-case hex) in a shared cache.

use std::{collections::HashMap, sync::Arc, time::Duration};

use tokio::sync::RwLock;

const POLL_INTERVAL: Duration = Duration::from_secs(15);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(10);

/// Enrichment fields from OpenSky for one aircraft.
#[derive(Debug, Clone, Default)]
pub struct OpenSkyInfo {
    pub callsign:       Option<String>,
    pub squawk:         Option<String>,
    pub origin_country: Option<String>,
}

pub type OpenSkyCache = Arc<RwLock<HashMap<String, OpenSkyInfo>>>;

pub fn new_cache() -> OpenSkyCache {
    Arc::new(RwLock::new(HashMap::new()))
}

/// Spawn a background task that polls OpenSky every 15 s.
/// `bbox` = (lamin, lomin, lamax, lomax) in decimal degrees.
pub fn spawn_opensky_task(cache: OpenSkyCache, bbox: (f64, f64, f64, f64)) {
    tokio::spawn(async move {
        let client = match reqwest::Client::builder()
            .timeout(REQUEST_TIMEOUT)
            .build()
        {
            Ok(c) => c,
            Err(e) => {
                tracing::error!("OpenSky: failed to build HTTP client: {e}");
                return;
            }
        };

        let (lamin, lomin, lamax, lomax) = bbox;
        let url = format!(
            "https://opensky-network.org/api/states/all\
             ?lamin={lamin}&lomin={lomin}&lamax={lamax}&lomax={lomax}"
        );

        tracing::info!("OpenSky poller started (bbox: lamin={lamin} lomin={lomin} lamax={lamax} lomax={lomax})");

        loop {
            match client.get(&url).send().await {
                Ok(resp) if resp.status().is_success() => {
                    match resp.json::<serde_json::Value>().await {
                        Ok(body) => {
                            let states = body.get("states")
                                .and_then(|s| s.as_array())
                                .cloned()
                                .unwrap_or_default();

                            let mut map = cache.write().await;
                            map.clear();
                            for sv in &states {
                                // OpenSky state vector field indices:
                                // 0 = icao24, 1 = callsign, 2 = origin_country, 14 = squawk
                                let Some(icao) = sv.get(0)
                                    .and_then(|v| v.as_str())
                                    .map(|s| s.trim().to_lowercase())
                                    .filter(|s| !s.is_empty())
                                else { continue };

                                let callsign = sv.get(1)
                                    .and_then(|v| v.as_str())
                                    .map(|s| s.trim().to_string())
                                    .filter(|s| !s.is_empty());

                                let origin_country = sv.get(2)
                                    .and_then(|v| v.as_str())
                                    .map(|s| s.trim().to_string())
                                    .filter(|s| !s.is_empty());

                                let squawk = sv.get(14)
                                    .and_then(|v| v.as_str())
                                    .map(|s| s.trim().to_string())
                                    .filter(|s| !s.is_empty());

                                map.insert(icao, OpenSkyInfo { callsign, squawk, origin_country });
                            }
                            tracing::debug!("OpenSky: cache refreshed ({} aircraft)", map.len());
                        }
                        Err(e) => tracing::warn!("OpenSky: JSON parse error: {e}"),
                    }
                }
                Ok(resp) => tracing::warn!("OpenSky: HTTP {}", resp.status()),
                Err(e)   => tracing::warn!("OpenSky: request error: {e}"),
            }

            tokio::time::sleep(POLL_INTERVAL).await;
        }
    });
}
