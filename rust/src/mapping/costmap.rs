#[derive(Clone, Debug, PartialEq)]
pub struct GridGeometry {
    pub frame: String,
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    /// ROS position xyz and quaternion xyzw.
    pub origin: [f64; 7],
}
impl GridGeometry {
    pub fn validate(&self) -> Result<(), String> {
        if frame_name(&self.frame).is_empty() {
            return Err("Costmap frame is empty".into());
        }
        if self.width == 0
            || self.height == 0
            || self.width > 4096
            || self.height > 4096
            || u64::from(self.width) * u64::from(self.height) > 4 * 1024 * 1024
        {
            return Err("Costmap capacity is 4096 per side, 4M cells".into());
        }
        if !self.resolution.is_finite()
            || self.resolution <= 0.0
            || self.origin.iter().any(|v| !v.is_finite())
        {
            return Err("Invalid costmap resolution/origin".into());
        }
        let norm = self.origin[3]
            .hypot(self.origin[4])
            .hypot(self.origin[5].hypot(self.origin[6]));
        if (norm - 1.0).abs() > 1e-3 {
            return Err("Origin quaternion is not normalized".into());
        }
        let extent = f64::from(self.width.max(self.height)) * self.resolution;
        let offset = self.origin[..3]
            .iter()
            .fold(0.0f64, |acc, v| acc.max(v.abs()));
        if extent + offset > f64::from(f32::MAX) {
            return Err("Costmap geometry exceeds headset coordinate range".into());
        }
        Ok(())
    }
}
fn frame_name(frame: &str) -> &str {
    frame.trim().trim_start_matches('/')
}
pub fn raw_to_occupancy(value: u8) -> i8 {
    match value {
        0 => 0,
        253 => 99,
        254 => 100,
        255 => -1,
        v => (1 + 97 * (u32::from(v) - 1) / 251) as i8,
    }
}
fn validate_cells(data: &[i8]) -> Result<(), String> {
    if data.iter().any(|v| !(-1..=100).contains(v)) {
        Err("Occupancy cells must be -1..100".into())
    } else {
        Ok(())
    }
}

#[derive(Default, Debug)]
pub struct CostmapState {
    geometry: Option<GridGeometry>,
    data: Vec<i8>,
    stamp_ns: u64,
    revision: u64,
}
impl CostmapState {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn geometry(&self) -> Option<&GridGeometry> {
        self.geometry.as_ref()
    }
    pub fn data(&self) -> &[i8] {
        &self.data
    }
    pub fn stamp_ns(&self) -> u64 {
        self.stamp_ns
    }
    pub fn revision(&self) -> u64 {
        self.revision
    }
    pub fn full(
        &mut self,
        geometry: GridGeometry,
        data: Vec<i8>,
        stamp_ns: u64,
    ) -> Result<bool, String> {
        geometry.validate()?;
        if data.len() as u64 != u64::from(geometry.width) * u64::from(geometry.height) {
            return Err("Costmap payload length mismatch".into());
        }
        validate_cells(&data)?;
        if self.geometry.is_some() && stamp_ns < self.stamp_ns {
            return Ok(false);
        }
        self.geometry = Some(geometry);
        self.data = data;
        self.stamp_ns = stamp_ns;
        self.revision += 1;
        Ok(true)
    }
    pub fn full_raw(
        &mut self,
        geometry: GridGeometry,
        data: &[u8],
        stamp_ns: u64,
    ) -> Result<bool, String> {
        geometry.validate()?;
        if data.len() as u64 != u64::from(geometry.width) * u64::from(geometry.height) {
            return Err("Costmap payload length mismatch".into());
        }
        self.full(
            geometry,
            data.iter().map(|v| raw_to_occupancy(*v)).collect(),
            stamp_ns,
        )
    }
    fn check_patch(
        &self,
        frame: &str,
        origin: [u32; 2],
        size: [u32; 2],
        len: usize,
        stamp: u64,
    ) -> Result<bool, String> {
        let grid = self
            .geometry
            .as_ref()
            .ok_or("Costmap snapshot required before patches")?;
        if stamp < self.stamp_ns {
            return Ok(false);
        }
        if frame_name(frame) != frame_name(&grid.frame) {
            return Err("Costmap patch frame differs".into());
        }
        if size.contains(&0)
            || u64::from(origin[0]) + u64::from(size[0]) > u64::from(grid.width)
            || u64::from(origin[1]) + u64::from(size[1]) > u64::from(grid.height)
        {
            return Err("Costmap patch outside snapshot".into());
        }
        if len as u64 != u64::from(size[0]) * u64::from(size[1]) {
            return Err("Costmap patch payload length mismatch".into());
        }
        Ok(true)
    }
    pub fn patch(
        &mut self,
        frame: &str,
        origin: [u32; 2],
        size: [u32; 2],
        data: &[i8],
        stamp_ns: u64,
    ) -> Result<bool, String> {
        if !self.check_patch(frame, origin, size, data.len(), stamp_ns)? {
            return Ok(false);
        }
        validate_cells(data)?;
        let stride = self.geometry.as_ref().unwrap().width as usize;
        let width = size[0] as usize;
        for (row, values) in data.chunks_exact(width).enumerate() {
            let start = (origin[1] as usize + row) * stride + origin[0] as usize;
            self.data[start..start + width].copy_from_slice(values);
        }
        self.stamp_ns = stamp_ns;
        self.revision += 1;
        Ok(true)
    }
    pub fn patch_raw(
        &mut self,
        frame: &str,
        origin: [u32; 2],
        size: [u32; 2],
        data: &[u8],
        stamp_ns: u64,
    ) -> Result<bool, String> {
        if !self.check_patch(frame, origin, size, data.len(), stamp_ns)? {
            return Ok(false);
        }
        let stride = self.geometry.as_ref().unwrap().width as usize;
        let width = size[0] as usize;
        for (row, values) in data.chunks_exact(width).enumerate() {
            let start = (origin[1] as usize + row) * stride + origin[0] as usize;
            for (out, value) in self.data[start..start + width].iter_mut().zip(values) {
                *out = raw_to_occupancy(*value);
            }
        }
        self.stamp_ns = stamp_ns;
        self.revision += 1;
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn geometry() -> GridGeometry {
        GridGeometry {
            frame: "map".into(),
            width: 3,
            height: 2,
            resolution: 0.05,
            origin: [0., 0., 0., 0., 0., 0., 1.],
        }
    }
    #[test]
    fn costmap_matches_python_encoding_and_patch_order() {
        let mut map = CostmapState::new();
        assert!(map.patch("map", [0, 0], [1, 1], &[0], 1).is_err());
        map.full_raw(geometry(), &[0, 1, 252, 253, 254, 255], 10)
            .unwrap();
        assert_eq!(map.data(), &[0, 1, 98, 99, 100, -1]);
        map.patch(" /map ", [1, 0], [1, 2], &[40, 50], 11).unwrap();
        assert_eq!(map.data(), &[0, 40, 98, 99, 50, -1]);
        assert!(!map.patch("map", [0, 0], [1, 1], &[12], 9).unwrap());
        assert_eq!(map.revision(), 2);
        for (origin, size, data) in [
            ([0, 0], [2, 1], vec![5, 101]),
            ([3, 0], [1, 1], vec![5]),
            ([u32::MAX, 0], [2, 1], vec![5, 5]),
        ] {
            assert!(map.patch("map", origin, size, &data, 12).is_err());
            assert_eq!(map.data(), &[0, 40, 98, 99, 50, -1]);
        }
    }
    #[test]
    fn invalid_geometry_is_atomic() {
        let mut map = CostmapState::new();
        map.full(geometry(), vec![0; 6], 5).unwrap();
        for resolution in [f64::NAN, f64::INFINITY, 0., -1., f64::MAX] {
            let mut invalid = geometry();
            invalid.resolution = resolution;
            assert!(map.full(invalid, vec![1; 6], 6).is_err());
        }
        let mut invalid = geometry();
        invalid.origin[6] = 0.;
        assert!(map.full(invalid, vec![1; 6], 6).is_err());
        assert_eq!(map.data(), &[0; 6]);
        assert_eq!(map.stamp_ns(), 5);
    }
}
