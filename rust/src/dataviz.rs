use crate::color::ColorManager;
use crate::core::types::{DataSourceType, SensorType, VisualizationType};
use crate::sensors::Sensor;
use std::collections::HashMap;

pub type RenderOptions = HashMap<String, serde_json::Value>;

#[derive(Debug, Clone)]
pub struct DataSource {
    pub name: String,
    pub source_type: DataSourceType,
    pub topic: String,
    pub frame_id: String,
    pub robot_name: Option<String>,
    pub metadata: HashMap<String, serde_json::Value>,
}

impl DataSource {
    pub fn is_robot_specific(&self) -> bool {
        self.robot_name.is_some()
    }
}

#[derive(Debug, Clone)]
pub struct VisualizationConfig {
    pub viz_type: VisualizationType,
    pub data_source: DataSource,
    pub display_name: String,
    pub enabled: bool,
    pub render_options: RenderOptions,
    pub layer_priority: i32,
}

impl VisualizationConfig {
    pub fn is_robot_specific(&self) -> bool {
        self.data_source.is_robot_specific()
    }
}

#[derive(Debug, Clone)]
pub struct DataViz {
    pub name: String,
    pub visualizations: Vec<VisualizationConfig>,
    pub color_manager: ColorManager,
}

impl DataViz {
    pub fn new(name: impl Into<String>) -> Self {
        let name = name.into();
        assert!(!name.is_empty(), "DataViz name cannot be empty");
        Self {
            name,
            visualizations: Vec::new(),
            color_manager: ColorManager::default(),
        }
    }

    pub fn add_sensor_visualization(
        &mut self,
        sensor: &dyn Sensor,
        robot_name: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        let viz_type = Self::viz_type_for_sensor(sensor.sensor_type());

        if !options.contains_key("color") {
            if viz_type == VisualizationType::LaserScan {
                let color = self.color_manager.get_laser_scan_color(robot_name, 0.8);
                options.insert("color".to_string(), serde_json::json!(color.to_hex()));
                options.insert("alpha".to_string(), serde_json::json!(color.a));
            } else {
                let color = self.color_manager.get_robot_color(robot_name);
                options.insert("color".to_string(), serde_json::json!(color.to_hex()));
            }
        }

        let data_source = DataSource {
            name: sensor.name().to_string(),
            source_type: DataSourceType::Sensor,
            topic: sensor.topic().to_string(),
            frame_id: sensor.frame_id().to_string(),
            robot_name: Some(robot_name.to_string()),
            metadata: sensor.metadata().clone(),
        };

        let config = VisualizationConfig {
            display_name: format!("{robot_name}:{} ({})", sensor.name(), viz_type.as_str()),
            viz_type,
            data_source,
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 0,
        };

        self.add_or_update_visualization(config);
    }

    pub fn add_robot_transform(
        &mut self,
        robot_name: &str,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        if !options.contains_key("color") {
            let color = self.color_manager.get_transform_color(robot_name, 1.0);
            options.insert("color".to_string(), serde_json::json!(color.to_hex()));
        }
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::CoordinateAxes,
            display_name: format!("{robot_name}:transform"),
            data_source: DataSource {
                name: format!("{robot_name}_transform"),
                source_type: DataSourceType::RobotTransform,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: Some(robot_name.to_string()),
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 0,
        });
    }

    pub fn add_robot_global_path(
        &mut self,
        robot_name: &str,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        if !options.contains_key("color") {
            let color = self.color_manager.get_path_color(robot_name, "global", 0.9);
            options.insert("color".to_string(), serde_json::json!(color.to_hex()));
            options.insert("alpha".to_string(), serde_json::json!(color.a));
            options.insert("line_width".to_string(), serde_json::json!(3));
        }
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::Path,
            display_name: format!("{robot_name}:global_path"),
            data_source: DataSource {
                name: format!("{robot_name}_global_path"),
                source_type: DataSourceType::RobotGlobalPath,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: Some(robot_name.to_string()),
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 3,
        });
    }

    pub fn add_robot_local_path(
        &mut self,
        robot_name: &str,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        if !options.contains_key("color") {
            let color = self.color_manager.get_path_color(robot_name, "local", 0.9);
            options.insert("color".to_string(), serde_json::json!(color.to_hex()));
            options.insert("alpha".to_string(), serde_json::json!(color.a));
            options.insert("line_width".to_string(), serde_json::json!(2));
            options.insert("line_style".to_string(), serde_json::json!("dashed"));
        }
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::Path,
            display_name: format!("{robot_name}:local_path"),
            data_source: DataSource {
                name: format!("{robot_name}_local_path"),
                source_type: DataSourceType::RobotLocalPath,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: Some(robot_name.to_string()),
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 4,
        });
    }

    pub fn add_robot_trajectory(
        &mut self,
        robot_name: &str,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        if !options.contains_key("color") {
            let base = self.color_manager.get_robot_color(robot_name);
            options.insert("color".to_string(), serde_json::json!(base.to_hex()));
            options.insert("alpha".to_string(), serde_json::json!(0.5));
            options.insert("line_width".to_string(), serde_json::json!(1));
        }
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::Trajectory,
            display_name: format!("{robot_name}:trajectory"),
            data_source: DataSource {
                name: format!("{robot_name}_trajectory"),
                source_type: DataSourceType::RobotTrajectory,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: Some(robot_name.to_string()),
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 1,
        });
    }

    pub fn add_robot_velocity_data(
        &mut self,
        robot_name: &str,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        options.entry("units".to_string()).or_insert_with(|| serde_json::json!("m/s"));
        options
            .entry("text_back_offset_m".to_string())
            .or_insert_with(|| serde_json::json!(0.36));
        options
            .entry("floor_offset_m".to_string())
            .or_insert_with(|| serde_json::json!(0.01));
        options
            .entry("update_hz".to_string())
            .or_insert_with(|| serde_json::json!(10.0));

        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::VelocityData,
            display_name: format!("{robot_name}:velocity"),
            data_source: DataSource {
                name: format!("{robot_name}_velocity"),
                source_type: DataSourceType::RobotVelocityData,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: Some(robot_name.to_string()),
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 2,
        });
    }

    pub fn add_robot_odometry_trail(
        &mut self,
        robot_name: &str,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        if !options.contains_key("color") {
            let color = self.color_manager.get_path_color(robot_name, "trajectory", 0.65);
            options.insert("color".to_string(), serde_json::json!(color.to_hex()));
            options.insert("alpha".to_string(), serde_json::json!(color.a));
        }
        options.entry("max_points".to_string()).or_insert_with(|| serde_json::json!(48));
        options
            .entry("history_seconds".to_string())
            .or_insert_with(|| serde_json::json!(3.2));
        options
            .entry("min_spacing_m".to_string())
            .or_insert_with(|| serde_json::json!(0.07));
        options
            .entry("line_width_m".to_string())
            .or_insert_with(|| serde_json::json!(0.0096));
        options
            .entry("trail_back_offset_m".to_string())
            .or_insert_with(|| serde_json::json!(0.44));

        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::OdometryTrail,
            display_name: format!("{robot_name}:odometry_trail"),
            data_source: DataSource {
                name: format!("{robot_name}_odometry_trail"),
                source_type: DataSourceType::RobotOdometryTrail,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: Some(robot_name.to_string()),
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 1,
        });
    }

    pub fn add_robot_collision_risk(
        &mut self,
        robot_name: &str,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        options
            .entry("threshold_m".to_string())
            .or_insert_with(|| serde_json::json!(0.45));
        options
            .entry("radius_m".to_string())
            .or_insert_with(|| serde_json::json!(0.45));
        options
            .entry("source".to_string())
            .or_insert_with(|| serde_json::json!("laser_scan"));
        options
            .entry("alpha_min".to_string())
            .or_insert_with(|| serde_json::json!(0.0));
        options
            .entry("alpha_max".to_string())
            .or_insert_with(|| serde_json::json!(0.55));

        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::CollisionRisk,
            display_name: format!("{robot_name}:collision_risk"),
            data_source: DataSource {
                name: format!("{robot_name}_collision_risk"),
                source_type: DataSourceType::RobotCollisionRisk,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: Some(robot_name.to_string()),
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 6,
        });
    }

    pub fn add_occupancy_grid(
        &mut self,
        topic: &str,
        frame_id: &str,
        render_options: Option<RenderOptions>,
    ) {
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::OccupancyGrid,
            display_name: "occupancy_grid".to_string(),
            data_source: DataSource {
                name: "occupancy_grid".to_string(),
                source_type: DataSourceType::OccupancyGrid,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: -10,
        });
    }

    pub fn add_3d_map(&mut self, topic: &str, frame_id: &str, render_options: Option<RenderOptions>) {
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::PointCloud,
            display_name: "map_3d".to_string(),
            data_source: DataSource {
                name: "map_3d".to_string(),
                source_type: DataSourceType::Map3D,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: -5,
        });
    }

    pub fn add_3d_mesh(&mut self, topic: &str, frame_id: &str, render_options: Option<RenderOptions>) {
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::Mesh,
            display_name: "map_3d_mesh".to_string(),
            data_source: DataSource {
                name: "map_3d_mesh".to_string(),
                source_type: DataSourceType::Map3D,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: -4,
        });
    }

    pub fn add_3d_octomap(&mut self, topic: &str, frame_id: &str, render_options: Option<RenderOptions>) {
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::Octomap,
            display_name: "map_3d_octomap".to_string(),
            data_source: DataSource {
                name: "map_3d_octomap".to_string(),
                source_type: DataSourceType::Octomap,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: -3,
        });
    }

    pub fn add_gaussian_splat_map(
        &mut self,
        manifest_topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
        preview_topic: &str,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        options
            .entry("manifest_topic".to_string())
            .or_insert_with(|| serde_json::json!(manifest_topic));
        options
            .entry("chunk_begin_topic".to_string())
            .or_insert_with(|| serde_json::json!("/horus/gaussian_splat/chunk_begin"));
        options
            .entry("chunk_item_topic".to_string())
            .or_insert_with(|| serde_json::json!("/horus/gaussian_splat/chunk_item"));
        options
            .entry("chunk_end_topic".to_string())
            .or_insert_with(|| serde_json::json!("/horus/gaussian_splat/chunk_end"));
        options.entry("asset_format".to_string()).or_insert_with(|| serde_json::json!("3dgs_ply"));
        options
            .entry("source_coordinate_space".to_string())
            .or_insert_with(|| serde_json::json!("colmap"));
        options.entry("render_mode".to_string()).or_insert_with(|| serde_json::json!("splats"));
        options.entry("max_splats".to_string()).or_insert_with(|| serde_json::json!(350000));
        options.entry("render_scale".to_string()).or_insert_with(|| serde_json::json!(0.5));
        options.entry("sh_order".to_string()).or_insert_with(|| serde_json::json!(2));
        options
            .entry("half_precision_sh".to_string())
            .or_insert_with(|| serde_json::json!(true));
        options.entry("adaptive_sort".to_string()).or_insert_with(|| serde_json::json!(true));
        options.entry("sort_passes".to_string()).or_insert_with(|| serde_json::json!(2));
        options.entry("opacity_scale".to_string()).or_insert_with(|| serde_json::json!(1.0));
        options.entry("splat_scale".to_string()).or_insert_with(|| serde_json::json!(1.0));
        options
            .entry("contribution_cull_threshold".to_string())
            .or_insert_with(|| serde_json::json!(0.1));
        options
            .entry("high_precision_rt".to_string())
            .or_insert_with(|| serde_json::json!(false));
        options
            .entry("pointcloud_fallback".to_string())
            .or_insert_with(|| serde_json::json!(true));
        options
            .entry("fallback_topic".to_string())
            .or_insert_with(|| serde_json::json!(preview_topic));
        options
            .entry("fallback_frame".to_string())
            .or_insert_with(|| serde_json::json!(frame_id));

        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::GaussianSplat,
            display_name: "gaussian_splat".to_string(),
            data_source: DataSource {
                name: "gaussian_splat".to_string(),
                source_type: DataSourceType::Map3D,
                topic: manifest_topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: -2,
        });
    }

    pub fn add_semantic_box(
        &mut self,
        semantic_id: &str,
        label: &str,
        center: (f32, f32, f32),
        size: (f32, f32, f32),
        frame_id: &str,
        rotation_offset_euler: Option<(f32, f32, f32)>,
    ) {
        let semantic_id = semantic_id.trim();
        let label = label.trim();
        if semantic_id.is_empty() || label.is_empty() {
            return;
        }

        let mut render_options = HashMap::new();
        render_options.insert("id".to_string(), serde_json::json!(semantic_id));
        render_options.insert("label".to_string(), serde_json::json!(label));
        render_options.insert(
            "center".to_string(),
            serde_json::json!({"x": center.0, "y": center.1, "z": center.2}),
        );
        render_options.insert(
            "size".to_string(),
            serde_json::json!({"x": size.0, "y": size.1, "z": size.2}),
        );
        let rot = rotation_offset_euler.unwrap_or((0.0, 0.0, 0.0));
        render_options.insert(
            "rotation_offset_euler".to_string(),
            serde_json::json!({"x": rot.0, "y": rot.1, "z": rot.2}),
        );

        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::SemanticBox,
            display_name: format!("semantic_box:{semantic_id}"),
            data_source: DataSource {
                name: format!("semantic_box_{semantic_id}"),
                source_type: DataSourceType::SemanticBox,
                topic: format!("/horus/semantic_boxes/{semantic_id}"),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options,
            layer_priority: 8,
        });
    }

    pub fn add_global_navigation_path(
        &mut self,
        topic: &str,
        frame_id: &str,
        mut render_options: Option<RenderOptions>,
    ) {
        let options = render_options.get_or_insert_with(HashMap::new);
        options
            .entry("color".to_string())
            .or_insert_with(|| serde_json::json!("#00FF00"));
        options
            .entry("line_width".to_string())
            .or_insert_with(|| serde_json::json!(4));
        options
            .entry("alpha".to_string())
            .or_insert_with(|| serde_json::json!(0.8));

        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::Path,
            display_name: "global_navigation_path".to_string(),
            data_source: DataSource {
                name: "global_navigation_path".to_string(),
                source_type: DataSourceType::GlobalNavigationPath,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 5,
        });
    }

    pub fn add_navigation_path(
        &mut self,
        topic: &str,
        frame_id: &str,
        render_options: Option<RenderOptions>,
    ) {
        self.add_global_navigation_path(topic, frame_id, render_options);
    }

    pub fn add_tf_tree(&mut self, topic: &str, frame_id: &str, render_options: Option<RenderOptions>) {
        self.add_or_update_visualization(VisualizationConfig {
            viz_type: VisualizationType::TransformTree,
            display_name: "tf_tree".to_string(),
            data_source: DataSource {
                name: "tf_tree".to_string(),
                source_type: DataSourceType::TfTree,
                topic: topic.to_string(),
                frame_id: frame_id.to_string(),
                robot_name: None,
                metadata: HashMap::new(),
            },
            enabled: true,
            render_options: render_options.unwrap_or_default(),
            layer_priority: 0,
        });
    }

    pub fn get_robot_visualizations(&self, robot_name: &str) -> Vec<VisualizationConfig> {
        self.visualizations
            .iter()
            .filter(|viz| viz.data_source.robot_name.as_deref() == Some(robot_name))
            .cloned()
            .collect()
    }

    pub fn get_global_visualizations(&self) -> Vec<VisualizationConfig> {
        self.visualizations
            .iter()
            .filter(|viz| !viz.is_robot_specific())
            .cloned()
            .collect()
    }

    pub fn get_visualizations_by_type(&self, viz_type: VisualizationType) -> Vec<VisualizationConfig> {
        self.visualizations
            .iter()
            .filter(|viz| viz.viz_type == viz_type)
            .cloned()
            .collect()
    }

    pub fn get_enabled_visualizations(&self) -> Vec<VisualizationConfig> {
        let mut enabled: Vec<_> = self
            .visualizations
            .iter()
            .filter(|viz| viz.enabled)
            .cloned()
            .collect();
        enabled.sort_by(|a, b| b.layer_priority.cmp(&a.layer_priority));
        enabled
    }

    pub fn enable_robot_visualizations(&mut self, robot_name: &str) {
        for viz in &mut self.visualizations {
            if viz.data_source.robot_name.as_deref() == Some(robot_name) {
                viz.enabled = true;
            }
        }
    }

    pub fn disable_robot_visualizations(&mut self, robot_name: &str) {
        for viz in &mut self.visualizations {
            if viz.data_source.robot_name.as_deref() == Some(robot_name) {
                viz.enabled = false;
            }
        }
    }

    pub fn remove_robot_visualizations(&mut self, robot_name: &str) -> usize {
        let before = self.visualizations.len();
        self.visualizations
            .retain(|viz| viz.data_source.robot_name.as_deref() != Some(robot_name));
        before - self.visualizations.len()
    }

    pub fn get_summary(&self) -> serde_json::Value {
        let mut by_robot: HashMap<String, usize> = HashMap::new();
        let mut by_type: HashMap<String, usize> = HashMap::new();
        let mut by_data_source: HashMap<String, usize> = HashMap::new();

        for viz in &self.visualizations {
            if let Some(robot_name) = &viz.data_source.robot_name {
                *by_robot.entry(robot_name.clone()).or_insert(0) += 1;
            }
            *by_type.entry(viz.viz_type.as_str().to_string()).or_insert(0) += 1;
            *by_data_source
                .entry(viz.data_source.source_type.as_str().to_string())
                .or_insert(0) += 1;
        }

        serde_json::json!({
            "name": self.name,
            "total_visualizations": self.visualizations.len(),
            "enabled_visualizations": self.get_enabled_visualizations().len(),
            "robot_specific": self.visualizations.iter().filter(|v| v.is_robot_specific()).count(),
            "global": self.visualizations.iter().filter(|v| !v.is_robot_specific()).count(),
            "by_robot": by_robot,
            "by_type": by_type,
            "by_data_source": by_data_source,
        })
    }

    pub fn add_or_update_visualization(&mut self, viz_config: VisualizationConfig) {
        if let Some(pos) = self.visualizations.iter().position(|existing| {
            existing.data_source.name == viz_config.data_source.name
                && existing.viz_type == viz_config.viz_type
        }) {
            self.visualizations[pos] = viz_config;
        } else {
            self.visualizations.push(viz_config);
        }
    }

    fn viz_type_for_sensor(sensor_type: SensorType) -> VisualizationType {
        match sensor_type {
            SensorType::Camera => VisualizationType::CameraFeed,
            SensorType::LaserScan => VisualizationType::LaserScan,
            SensorType::Lidar3D => VisualizationType::PointCloud,
            _ => VisualizationType::Markers,
        }
    }
}
