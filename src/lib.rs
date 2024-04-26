mod servers;
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=Object, init)]
pub struct RapierPhysics2DExtensionLibrary {
}

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics2DExtensionLibrary {

    fn min_level() -> InitLevel {
        InitLevel::Servers
    }
    fn on_level_init(level: InitLevel) {
        match level {
            InitLevel::Servers => {
                servers::register();
            }
            _ => (),
        }
        godot_print!("[Rust]      Init level {:?}", level);
    }

    fn on_level_deinit(level: InitLevel) {
        match level {
            InitLevel::Servers => {
                servers::unregister();
            }
            _ => (),
        }
        godot_print!("[Rust]      Deinit level {:?}", level);
    }
}
