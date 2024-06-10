pub mod rapier_area;
pub mod rapier_body;
pub mod rapier_collision_object;
#[cfg(feature = "dim2")]
pub mod rapier_direct_body_state_2d;
#[cfg(feature = "dim3")]
pub mod rapier_direct_body_state_3d;
#[cfg(feature = "dim2")]
pub type RapierDirectBodyState = rapier_direct_body_state_2d::RapierDirectBodyState2D;
#[cfg(feature = "dim2")]
pub type PhysicsDirectBodyState = godot::engine::PhysicsDirectBodyState2D;
#[cfg(feature = "dim3")]
pub type RapierDirectBodyState = rapier_direct_body_state_3d::RapierDirectBodyState3D;
#[cfg(feature = "dim3")]
pub type PhysicsDirectBodyState = godot::engine::PhysicsDirectBodyState3D;
