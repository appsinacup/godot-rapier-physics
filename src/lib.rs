use godot::prelude::*;

struct RapierPhysicsExtension;


#[derive(GodotClass)]
#[class(base=Object,init)]
pub struct RapierPhysicsServerFactory;

#[godot_api]
impl RapierPhysicsServerFactory {
    #[func]
    fn create_server_3D() -> Gd<RapierPhysicsServer3D> {
        Gd::<RapierPhysicsServer3D>::new_default()
    }
    #[func]
    fn create_server_2D() -> Gd<RapierPhysicsServer2D> {
        Gd::<RapierPhysicsServer2D>::new_default()
    }
}

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysicsExtension {
    fn on_level_init(_level: InitLevel) {
        crate::auto_register_classes();
        let mut manager = PhysicsServer3DManager::singleton();
        let initializer = Gd::<RapierPhysicsServerFactory>::new_default();
        manager.register_server("Rapier3D".into(), initializer.callable("create_server_3D"));
        manager.register_server("Rapier2D".into(), initializer.callable("create_server_2D"));
        self.initializer = Some(initializer);
        println!("[Rust]      Init level {:?}", _level);
    }

    fn on_level_deinit(_level: InitLevel) {
        println!("[Rust]      Deinit level {:?}", _level);
    }
}
