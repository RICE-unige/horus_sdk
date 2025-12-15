use crate::core::types::ColorScheme;
use std::collections::HashMap;

/// RGB color representation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RGBColor {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: f32,
}

impl RGBColor {
    pub fn new(r: u8, g: u8, b: u8, a: f32) -> Self {
        Self { r, g, b, a }
    }

    pub fn to_hex(&self) -> String {
        format!("#{:02x}{:02x}{:02x}", self.r, self.g, self.b)
    }

    pub fn from_hex(hex: &str, alpha: f32) -> Result<Self, String> {
        let hex = hex.trim_start_matches('#');
        if hex.len() != 6 {
            return Err("Hex color must be 6 characters".to_string());
        }

        let r = u8::from_str_radix(&hex[0..2], 16).map_err(|e| e.to_string())?;
        let g = u8::from_str_radix(&hex[2..4], 16).map_err(|e| e.to_string())?;
        let b = u8::from_str_radix(&hex[4..6], 16).map_err(|e| e.to_string())?;

        Ok(Self { r, g, b, a: alpha })
    }
}

/// Manages unique color assignment for robots and their visualizations
pub struct ColorManager {
    robot_colors: HashMap<String, RGBColor>,
    color_scheme: ColorScheme,
    color_index: usize,
}

impl ColorManager {
    pub fn new(color_scheme: ColorScheme) -> Self {
        Self {
            robot_colors: HashMap::new(),
            color_scheme,
            color_index: 0,
        }
    }

    pub fn get_robot_color(&mut self, robot_name: &str) -> RGBColor {
        if let Some(color) = self.robot_colors.get(robot_name) {
            *color
        } else {
            let color = self.assign_new_color(robot_name);
            self.robot_colors.insert(robot_name.to_string(), color);
            color
        }
    }

    fn assign_new_color(&mut self, robot_name: &str) -> RGBColor {
        // Simple color generation based on index
        let palette = self.get_palette();
        let color = if self.color_index < palette.len() {
            let hex = palette[self.color_index];
            self.color_index += 1;
            RGBColor::from_hex(hex, 1.0).unwrap_or(RGBColor::new(255, 0, 0, 1.0))
        } else {
            // Generate deterministic color from robot name
            self.generate_deterministic_color(robot_name)
        };
        color
    }

    fn get_palette(&self) -> Vec<&'static str> {
        match self.color_scheme {
            ColorScheme::Bright => vec![
                "#FF0000", "#00FF00", "#0000FF", "#FF7F00", "#FF00FF", 
                "#00FFFF", "#FFFF00", "#FF007F", "#7F00FF", "#00FF7F",
            ],
            _ => vec!["#FF0000", "#00FF00", "#0000FF"],
        }
    }

    fn generate_deterministic_color(&self, robot_name: &str) -> RGBColor {
        let hash = md5::compute(robot_name.as_bytes());
        let r = hash[0];
        let g = hash[1];
        let b = hash[2];
        RGBColor::new(r, g, b, 1.0)
    }
}
