pub mod body;
pub mod collider;
pub mod convert;
pub mod event_handler;
pub mod fluid;
pub mod handle;
pub mod joint;
pub mod physics_hooks;
pub mod physics_world;
pub mod prelude;
pub mod query;
pub mod settings;
pub mod shape;
pub mod user_data;
#[cfg(feature = "dim2")]
pub const ANG_ZERO: rapier::math::Real = 0.0;
#[cfg(feature = "dim3")]
pub const ANG_ZERO: rapier::na::Vector3<rapier::math::Real> =
    rapier::na::Vector3::new(0.0, 0.0, 0.0);
