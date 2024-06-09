pub mod rapier_area;
pub mod rapier_body;
pub mod rapier_collision_object;
#[cfg(feature = "dim2")]
pub mod rapier_direct_body_state_2d;
#[cfg(feature = "dim3")]
pub mod rapier_direct_body_state_3d;

#[cfg(feature = "dim2")]
pub type PhysicsDirectBodyState = rapier_direct_body_state_2d::PhysicsDirectBodyState2D;
#[cfg(feature = "dim3")]
pub type PhysicsDirectBodyState = rapier_direct_body_state_3d::PhysicsDirectBodyState3D;
