#![feature(try_blocks)]
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
extern crate salva2d_f64 as salva;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate salva3d as salva;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate salva3d_f64 as salva;
mod bodies;
mod fluids;
mod joints;
mod nodes;
mod rapier_wrapper;
mod servers;
mod shapes;
mod spaces;
mod types;
use godot::prelude::*;
#[cfg(feature = "dim2")]
#[derive(GodotClass)]
#[class(base=Object, init)]
/// Used to register the Rapier 2D extension library.
pub struct RapierPhysics2DExtensionLibrary {}
#[cfg(feature = "dim2")]
#[gdextension(entry_symbol = rapier_2d_init)]
unsafe impl ExtensionLibrary for RapierPhysics2DExtensionLibrary {
    fn min_level() -> InitLevel {
        InitLevel::Servers
    }

    fn on_stage_init(level: InitStage) {
        match level {
            InitStage::Scene => {
                servers::register_scene();
            }
            InitStage::Servers => {
                servers::register_server();
            }
            _ => (),
        }
    }

    fn on_stage_deinit(_level: InitStage) {}
}
#[cfg(feature = "dim3")]
#[derive(GodotClass)]
#[class(base=Object, init)]
/// Used to register the Rapier 3D extension library.
pub struct RapierPhysics3DExtensionLibrary {}
#[cfg(feature = "dim3")]
#[gdextension(entry_symbol = rapier_3d_init)]
unsafe impl ExtensionLibrary for RapierPhysics3DExtensionLibrary {
    fn min_level() -> InitLevel {
        InitLevel::Servers
    }

    fn on_stage_init(level: InitStage) {
        match level {
            InitStage::Scene => {
                servers::register_scene();
            }
            InitStage::Servers => {
                servers::register_server();
            }
            _ => (),
        }
    }

    fn on_stage_deinit(_level: InitStage) {}
}
