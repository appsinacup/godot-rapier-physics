#![feature(map_many_mut)]
mod bodies;
mod fluids;
mod joints;
mod rapier_wrapper;
mod servers2d;
//mod servers3d;
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
                servers2d::register_scene();
                //servers3d::register_scene();
            }
            InitLevel::Servers => {
                servers2d::register_server();
            }
            _ => (),
        }
    }

    fn on_level_deinit(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                servers2d::unregister_scene();
            }
            InitLevel::Servers => {
                servers2d::unregister_server();
            }
            _ => (),
        }
    }

    fn override_hot_reload() -> Option<bool> {
        Some(true)
    }
}
