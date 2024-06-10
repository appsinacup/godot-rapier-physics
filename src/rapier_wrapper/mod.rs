pub mod body;
pub mod collider;
pub mod convert;
pub mod event_handler;
pub mod fluid;
pub mod handle;
pub mod joint;
pub mod physics_hooks;
pub mod physics_world;
pub mod query;
pub mod settings;
pub mod shape;
#[cfg(feature = "dim2")]
pub mod shape_2d;
pub mod user_data;
#[cfg(feature = "dim2")]
pub const AngZERO: rapier::math::Real = 0.0;
#[cfg(feature = "dim3")]
pub const AngZERO: rapier::math::Vector3<Real> = zeroes();
