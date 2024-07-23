#![feature(map_many_mut)]
#![feature(let_chains)]
#![feature(try_blocks)]
#![feature(trait_alias)]
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
mod bodies;
mod fluids;
mod joints;
mod rapier_wrapper;
mod servers;
mod shapes;
mod spaces;
mod types;
use godot::prelude::*;
#[cfg(feature = "dim2")]
#[derive(GodotClass)]
#[class(base=Object, init)]
pub struct RapierPhysics2DExtensionLibrary {}
#[cfg(feature = "dim2")]
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

    fn on_level_deinit(_level: InitLevel) {}
}
#[cfg(feature = "dim3")]
#[derive(GodotClass)]
#[class(base=Object, init)]
pub struct RapierPhysics3DExtensionLibrary {}
#[cfg(feature = "dim3")]
#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics3DExtensionLibrary {
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

    fn on_level_deinit(_level: InitLevel) {}
}
