use crate::core::types::ColorScheme;
use std::collections::HashMap;

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

    pub fn to_hex(self) -> String {
        format!("#{:02x}{:02x}{:02x}", self.r, self.g, self.b)
    }

    pub fn to_rgba_tuple(self) -> (u8, u8, u8, f32) {
        (self.r, self.g, self.b, self.a)
    }

    pub fn to_normalized_tuple(self) -> (f32, f32, f32, f32) {
        (
            f32::from(self.r) / 255.0,
            f32::from(self.g) / 255.0,
            f32::from(self.b) / 255.0,
            self.a,
        )
    }

    pub fn from_hex(hex: &str, alpha: f32) -> Result<Self, String> {
        let normalized = hex.trim().trim_start_matches('#');
        if normalized.len() != 6 {
            return Err("Hex color must be 6 characters".to_string());
        }
        let r = u8::from_str_radix(&normalized[0..2], 16).map_err(|e| e.to_string())?;
        let g = u8::from_str_radix(&normalized[2..4], 16).map_err(|e| e.to_string())?;
        let b = u8::from_str_radix(&normalized[4..6], 16).map_err(|e| e.to_string())?;
        Self::validate(r, g, b, alpha)?;
        Ok(Self { r, g, b, a: alpha })
    }

    pub fn from_hsv(h: f32, s: f32, v: f32, alpha: f32) -> Self {
        let hh = ((h % 360.0) + 360.0) % 360.0;
        let c = v * s;
        let x = c * (1.0 - (((hh / 60.0) % 2.0) - 1.0).abs());
        let m = v - c;
        let (r1, g1, b1) = if hh < 60.0 {
            (c, x, 0.0)
        } else if hh < 120.0 {
            (x, c, 0.0)
        } else if hh < 180.0 {
            (0.0, c, x)
        } else if hh < 240.0 {
            (0.0, x, c)
        } else if hh < 300.0 {
            (x, 0.0, c)
        } else {
            (c, 0.0, x)
        };
        Self {
            r: ((r1 + m) * 255.0).round() as u8,
            g: ((g1 + m) * 255.0).round() as u8,
            b: ((b1 + m) * 255.0).round() as u8,
            a: alpha,
        }
    }

    fn validate(r: u8, g: u8, b: u8, a: f32) -> Result<(), String> {
        let _ = (r, g, b);
        if !(0.0..=1.0).contains(&a) {
            return Err("Alpha value must be between 0.0 and 1.0".to_string());
        }
        Ok(())
    }
}

impl std::fmt::Display for RGBColor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_hex())
    }
}

#[derive(Debug, Clone)]
pub struct ColorManager {
    color_scheme: ColorScheme,
    robot_colors: HashMap<String, RGBColor>,
    used_colors: Vec<String>,
    color_index: usize,
    palette: Vec<String>,
}

impl Default for ColorManager {
    fn default() -> Self {
        Self::new(ColorScheme::Bright)
    }
}

impl ColorManager {
    pub fn new(color_scheme: ColorScheme) -> Self {
        let palette = Self::palette_for_scheme(color_scheme);
        Self {
            color_scheme,
            robot_colors: HashMap::new(),
            used_colors: Vec::new(),
            color_index: 0,
            palette,
        }
    }

    pub fn color_scheme(&self) -> ColorScheme {
        self.color_scheme
    }

    pub fn get_robot_color(&mut self, robot_name: &str) -> RGBColor {
        if let Some(color) = self.robot_colors.get(robot_name) {
            return *color;
        }
        let color = self.assign_new_color(robot_name);
        self.robot_colors.insert(robot_name.to_string(), color);
        color
    }

    pub fn get_laser_scan_color(&mut self, robot_name: &str, alpha: f32) -> RGBColor {
        let base = self.get_robot_color(robot_name);
        RGBColor::new(base.r, base.g, base.b, alpha)
    }

    pub fn get_path_color(&mut self, robot_name: &str, path_type: &str, alpha: f32) -> RGBColor {
        let base = self.get_robot_color(robot_name);
        if path_type == "local" {
            RGBColor::new(
                base.r.saturating_add(50),
                base.g.saturating_add(50),
                base.b.saturating_add(50),
                (alpha * 0.7).min(1.0),
            )
        } else {
            RGBColor::new(base.r, base.g, base.b, alpha)
        }
    }

    pub fn get_transform_color(&mut self, robot_name: &str, alpha: f32) -> RGBColor {
        let base = self.get_robot_color(robot_name);
        RGBColor::new(base.r, base.g, base.b, alpha)
    }

    pub fn set_robot_color(&mut self, robot_name: &str, color: RGBColor) {
        self.robot_colors.insert(robot_name.to_string(), color);
    }

    pub fn reset_robot_color(&mut self, robot_name: &str) {
        self.robot_colors.remove(robot_name);
    }

    pub fn clear_all_colors(&mut self) {
        self.robot_colors.clear();
        self.used_colors.clear();
        self.color_index = 0;
    }

    pub fn get_all_robot_colors(&self) -> HashMap<String, RGBColor> {
        self.robot_colors.clone()
    }

    pub fn get_color_summary(&self) -> HashMap<String, String> {
        self.robot_colors
            .iter()
            .map(|(robot, color)| (robot.clone(), color.to_hex()))
            .collect()
    }

    fn assign_new_color(&mut self, robot_name: &str) -> RGBColor {
        if self.color_index < self.palette.len() {
            let color_hex = self.palette[self.color_index].clone();
            self.color_index += 1;
            self.used_colors.push(color_hex.clone());
            return RGBColor::from_hex(&color_hex, 1.0).unwrap_or(RGBColor::new(255, 0, 0, 1.0));
        }
        self.generate_deterministic_color(robot_name)
    }

    fn generate_deterministic_color(&self, robot_name: &str) -> RGBColor {
        let digest = md5::compute(robot_name.as_bytes());
        let mut r = digest[0];
        let mut g = digest[1];
        let mut b = digest[2];
        let min_brightness = 100u8;
        if r.saturating_add(g).saturating_add(b) < min_brightness.saturating_mul(3) {
            r = r.max(min_brightness);
            g = g.max(min_brightness);
            b = b.max(min_brightness);
        }
        RGBColor::new(r, g, b, 1.0)
    }

    fn palette_for_scheme(scheme: ColorScheme) -> Vec<String> {
        match scheme {
            ColorScheme::Bright => vec![
                "#FF0000", "#00FF00", "#0000FF", "#FF7F00", "#FF00FF", "#00FFFF", "#FFFF00",
                "#FF007F", "#7F00FF", "#00FF7F",
            ]
            .into_iter()
            .map(str::to_string)
            .collect(),
            ColorScheme::Pastel => vec![
                "#FFB3BA", "#FFDFBA", "#FFFFBA", "#BAFFC9", "#BAE1FF", "#C9BAFF", "#FFBAE1",
                "#E1BAFF", "#BAFFE1", "#FFE1BA",
            ]
            .into_iter()
            .map(str::to_string)
            .collect(),
            ColorScheme::Dark => vec![
                "#8B0000", "#006400", "#00008B", "#FF8C00", "#8B008B", "#008B8B", "#B8860B",
                "#8B4513", "#2F4F4F", "#800080",
            ]
            .into_iter()
            .map(str::to_string)
            .collect(),
            ColorScheme::Rainbow => (0..12)
                .map(|i| RGBColor::from_hsv((i as f32) * 30.0, 1.0, 1.0, 1.0).to_hex())
                .collect(),
            ColorScheme::Neon => (0..10)
                .map(|i| RGBColor::from_hsv((i as f32) * 36.0, 1.0, 1.0, 1.0).to_hex())
                .collect(),
        }
    }
}
