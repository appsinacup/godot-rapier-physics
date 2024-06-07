#[cfg(all(feature = "single", feature = "dim2"))]
extern crate rapier2d as rapier;
#[cfg(all(feature = "double", feature = "dim2"))]
extern crate rapier2d_f64 as rapier;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate rapier2d_f64 as rapier;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate rapier2d_f64 as rapier;
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
pub mod user_data;
