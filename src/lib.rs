mod bodies;
mod fluids;
mod joints;
mod rapier2d;
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
        InitLevel::Scene
    }
    fn on_level_init(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                servers::register_scene();
            //}
            //InitLevel::Servers => {
                servers::register_server();
            }
            _ => (),
        }
    }

    fn on_level_deinit(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                servers::unregister_scene();
            //}
            //InitLevel::Servers => {
                servers::unregister_server();
            }
            _ => (),
        }
    }
}
