mod servers;
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=Object, init)]
pub struct RapierPhysics2DExtensionLibrary {
}

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics2DExtensionLibrary {

    fn min_level() -> InitLevel {
        InitLevel::Scene
    }
    fn on_level_init(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                servers::register();
            }
            _ => (),
        }
    }

    fn on_level_deinit(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                servers::unregister();
            }
            _ => (),
        }
    }
}
