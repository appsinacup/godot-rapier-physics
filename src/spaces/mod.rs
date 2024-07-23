#[cfg(feature = "dim2")]
pub mod rapier_direct_space_state_2d;
#[cfg(feature = "dim3")]
pub mod rapier_direct_space_state_3d;
#[cfg(feature = "dim2")]
pub type RapierDirectSpaceState = rapier_direct_space_state_2d::RapierDirectSpaceState2D;
#[cfg(feature = "dim2")]
pub type PhysicsDirectSpaceState = godot::classes::PhysicsDirectSpaceState2D;
#[cfg(feature = "dim3")]
pub type RapierDirectSpaceState = rapier_direct_space_state_3d::RapierDirectSpaceState3D;
#[cfg(feature = "dim3")]
pub type PhysicsDirectSpaceState = godot::classes::PhysicsDirectSpaceState3D;
pub mod rapier_direct_space_state_impl;
pub mod rapier_space;
pub mod rapier_space_body_helper;
pub mod rapier_space_callbacks;
