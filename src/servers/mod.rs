use godot::{engine::{PhysicsServer2DManager}, prelude::*};

use crate::servers::rapier_physics_server_factory_2d::*;

mod rapier_physics_server_2d;
mod rapier_physics_server_factory_2d;

pub fn register() {
    let mut manager = PhysicsServer2DManager::singleton();
    let factory = RapierPhysicsServerFactory2D::new_alloc();
    manager.register_server("Rapier2D".into(), factory.callable("create_server"));
}

pub fn unregister() {
    // TODO
}
