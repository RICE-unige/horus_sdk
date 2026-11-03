//! Native map operations. Geometry is preserved; capacity errors are explicit.
mod costmap;
mod costmap_adapter;
pub use costmap_adapter::{CostmapAdapter,CostmapAdapterOptions};
mod live_mesh;
mod live_mesh_publisher;
mod mesh_page_scheduler;
mod mesh_protocol;
mod mesh_source_adapter;
mod mesh_source;
pub use mesh_source::{MeshSourceState,MeshSourceOptions,MeshPose,TriangleMarker,NvbloxBlockUpdate};
pub use mesh_source_adapter::{MeshSourceAdapter, MeshSourceAdapterOptions};
pub use costmap::{raw_to_occupancy, CostmapState, GridGeometry};
pub use live_mesh::{LiveMeshOptions, LiveMeshState, MeshPageUpdate};
pub use live_mesh_publisher::{LiveMeshPublisher, LiveMeshPublisherOptions};
pub use mesh_page_scheduler::{MeshPageScheduler, MeshSchedulerOptions};
pub use mesh_protocol::{
    decode_live_header, encode_live_payload, BlockGeometry, LiveHeader, MeshOperation,
    HMSH_V2_HEADER_SIZE,
};
