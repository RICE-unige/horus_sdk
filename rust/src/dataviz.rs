/// Data visualization system for robot sensors and environmental data
#[derive(Debug, Clone)]
pub struct DataViz {
    pub name: String,
}

impl DataViz {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
        }
    }
}
