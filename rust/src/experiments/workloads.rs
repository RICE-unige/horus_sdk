use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use std::path::Path;
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct CameraWorkload {
    pub streams: u32,
    pub resolution: String,
    pub fps: f64,
    pub encoding: String,
    pub bitrate_mbps: f64,
}
impl Default for CameraWorkload {
    fn default() -> Self {
        Self {
            streams: 0,
            resolution: String::new(),
            fps: 0.,
            encoding: "none".into(),
            bitrate_mbps: 0.,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct PointCloudWorkload {
    pub points: u64,
    pub hz: f64,
    pub point_step: u32,
    pub xyzrgb: bool,
}
impl Default for PointCloudWorkload {
    fn default() -> Self {
        Self {
            points: 0,
            hz: 0.,
            point_step: 16,
            xyzrgb: true,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct MapWorkload {
    pub representation: String,
    pub chunks: u64,
    pub vertices: u64,
    pub triangles: u64,
    pub hz: f64,
}
impl Default for MapWorkload {
    fn default() -> Self {
        Self {
            representation: "none".into(),
            chunks: 0,
            vertices: 0,
            triangles: 0,
            hz: 0.,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct WorkloadConfig {
    pub experiment: String,
    pub condition: String,
    pub duration_s: f64,
    pub warmup_s: f64,
    pub repetition: u32,
    pub robot_count: u32,
    pub operator_count: u32,
    pub transport: String,
    pub topology: String,
    pub robot_profile: String,
    pub map_profile: String,
    pub camera: CameraWorkload,
    pub pointcloud: PointCloudWorkload,
    pub map: MapWorkload,
    pub extra: Map<String, Value>,
}
impl Default for WorkloadConfig {
    fn default() -> Self {
        Self {
            experiment: "E0_baseline".into(),
            condition: "default".into(),
            duration_s: 120.,
            warmup_s: 30.,
            repetition: 1,
            robot_count: 1,
            operator_count: 1,
            transport: "ros2".into(),
            topology: "lan".into(),
            robot_profile: "minimal".into(),
            map_profile: "none".into(),
            camera: Default::default(),
            pointcloud: Default::default(),
            map: Default::default(),
            extra: Map::new(),
        }
    }
}
impl WorkloadConfig {
    pub fn stream_count(&self) -> u64 {
        u64::from(self.camera.streams)
            + if self.pointcloud.points > 0 && self.pointcloud.hz > 0. {
                u64::from(self.robot_count)
            } else {
                0
            }
            + if self.map.representation != "none" && self.map.hz > 0. {
                u64::from(self.robot_count)
            } else {
                0
            }
    }
    pub fn resolution(&self) -> &str {
        &self.camera.resolution
    }
    pub fn target_fps(&self) -> f64 {
        self.camera.fps
    }
    pub fn to_dict(&self) -> Result<Value> {
        Ok(serde_json::to_value(self)?)
    }
}
pub fn workload_from_dict(payload: &Value) -> Result<WorkloadConfig> {
    let mut result: WorkloadConfig = serde_json::from_value(payload.clone())?;
    let known = serde_json::to_value(WorkloadConfig::default())?;
    for (key, value) in payload
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("Workload must be an object"))?
    {
        if known.get(key).is_none() {
            result.extra.insert(key.clone(), value.clone());
        }
    }
    Ok(result)
}
pub fn load_workload_config(path: impl AsRef<Path>) -> Result<WorkloadConfig> {
    ensure!(
        std::fs::metadata(path.as_ref())?.len() <= 1024 * 1024,
        "Workload exceeds 1 MiB"
    );
    workload_from_dict(&serde_json::from_slice(&std::fs::read(path)?)?)
}
