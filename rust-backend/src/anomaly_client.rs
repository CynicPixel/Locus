// HTTP client for the Python LSTM anomaly classification service.
// Calls POST /classify with a window of 20 Kalman track states.
// Rate-limited per aircraft to one call per 5 seconds.

use serde::{Deserialize, Serialize};

#[derive(Serialize)]
struct KalmanStatePoint {
    lat: f64,
    lon: f64,
    alt_m: f64,
    vel_lat: f64,
    vel_lon: f64,
    vel_alt: f64,
}

#[derive(Serialize)]
struct ClassifyRequest<'a> {
    icao24: &'a str,
    states: Vec<KalmanStatePoint>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ClassifyResponse {
    #[allow(dead_code)]
    pub icao24: String,
    /// 0 = normal, 1 = anomalous
    pub anomaly_label: String,
    pub confidence: f64,
}

pub struct AnomalyClient {
    pub client: reqwest::Client,
    pub base_url: String,
}

impl AnomalyClient {
    pub fn new(base_url: String) -> Self {
        Self {
            client: reqwest::Client::new(),
            base_url,
        }
    }

    /// Classify a track history.  `history` contains tuples
    /// `(lat, lon, alt_m, vel_lat, vel_lon, vel_alt)`.
    pub async fn classify(
        &self,
        icao24: &str,
        history: &[(f64, f64, f64, f64, f64, f64)],
    ) -> anyhow::Result<ClassifyResponse> {
        let states: Vec<KalmanStatePoint> = history
            .iter()
            .map(|&(lat, lon, alt_m, vel_lat, vel_lon, vel_alt)| KalmanStatePoint {
                lat,
                lon,
                alt_m,
                vel_lat,
                vel_lon,
                vel_alt,
            })
            .collect();

        let resp = self
            .client
            .post(format!("{}/classify", self.base_url))
            .json(&ClassifyRequest { icao24, states })
            .timeout(std::time::Duration::from_millis(500))
            .send()
            .await?
            .json::<ClassifyResponse>()
            .await?;

        Ok(resp)
    }
}
