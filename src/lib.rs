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
#[cfg(feature = "single")]
pub type PackedFloatArray = PackedFloat32Array;
#[cfg(feature = "double")]
pub type PackedFloatArray = PackedFloat64Array;
#[cfg(feature = "dim3")]
pub type PackedVectorArray = godot::prelude::PackedVector3Array;
#[cfg(feature = "dim2")]
pub type PackedVectorArray = godot::prelude::PackedVector2Array;
#[cfg(feature = "dim3")]
pub type Transform = godot::prelude::Transform3D;
#[cfg(feature = "dim2")]
pub type Transform = godot::prelude::Transform2D;
#[cfg(feature = "dim3")]
pub type Vector = godot::prelude::Vector3;
#[cfg(feature = "dim2")]
pub type Vector = godot::prelude::Vector2;
#[cfg(feature = "dim3")]
pub type Angle = godot::prelude::Vector3;
#[cfg(feature = "dim2")]
pub type Angle = real;
#[cfg(feature = "dim3")]
pub type Rect = godot::prelude::Aabb;
#[cfg(feature = "dim2")]
pub type Rect = godot::prelude::Rect2;
#[cfg(feature = "dim2")]
pub const ANGLE_ZERO: rapier::math::Real = 0.0;
#[cfg(feature = "dim3")]
pub const ANGLE_ZERO: Vector3 = Vector3::new(0.0, 0.0, 0.0);
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
}
