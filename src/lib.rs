#![feature(map_many_mut)]

#[cfg(all(feature = "single", feature = "dim2"))]
extern crate rapier2d as rapier;
#[cfg(all(feature = "double", feature = "dim2"))]
extern crate rapier2d_f64 as rapier;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate rapier3d as rapier;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate rapier3d_f64 as rapier;

#[cfg(all(feature = "single", feature = "dim2"))]
extern crate salva2d as salva;
#[cfg(all(feature = "double", feature = "dim2"))]
extern crate salva2d as salva;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate salva3d as salva;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate salva3d as salva;

#[cfg(feature = "dim2")]
pub type Transform = Transform2D;
#[cfg(feature = "dim2")]
pub type Rect = Rect2;
#[cfg(feature = "dim3")]
pub type Transform = Transform3D;
#[cfg(feature = "dim3")]
pub type Rect = Aabb;

#[cfg(feature = "dim2")]
pub type Vector = Vector2;
#[cfg(feature = "dim2")]
pub type PackedVectorArray = PackedVector2Array;
#[cfg(feature = "dim2")]
pub type Angle = real;

#[cfg(feature = "dim3")]
pub type Vector = Vector3;
#[cfg(feature = "dim3")]
pub type PackedVectorArray = PackedVector3Array;
#[cfg(feature = "dim3")]
pub type Angle = Vector3;

mod bodies;
mod fluids;
mod joints;
mod rapier_wrapper;
mod servers;
mod shapes;
mod spaces;
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=Object, init)]
pub struct RapierPhysics2DExtensionLibrary {}

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics2DExtensionLibrary {
    fn min_level() -> InitLevel {
        InitLevel::Servers
    }
    fn on_level_init(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                servers::register_scene();
            }
            InitLevel::Servers => {
                servers::register_server();
            }
            _ => (),
        }
    }

    fn on_level_deinit(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                servers::unregister_scene();
            }
            InitLevel::Servers => {
                servers::unregister_server();
            }
            _ => (),
        }
    }

    fn override_hot_reload() -> Option<bool> {
        Some(true)
    }
}
