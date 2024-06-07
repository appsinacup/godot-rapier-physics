use godot::{engine::PhysicsServer3DManager, prelude::*};

use crate::servers3d::rapier_physics_server_factory_3d::*;
use crate::servers3d::rapier_project_settings_3d::RapierProjectSettings3D;

pub mod rapier_physics_server_3d;
pub mod rapier_physics_server_factory_3d;
pub mod rapier_physics_singleton_3d;
pub mod rapier_project_settings_3d;

pub fn register_server() {
    let mut manager = PhysicsServer3DManager::singleton();
    let factory = RapierPhysicsServerFactory3D::new_alloc();
    manager.register_server("Rapier3D".into(), factory.callable("create_server"));
}

pub fn register_scene() {
    RapierProjectSettings3D::register_settings();
}

pub fn unregister_server() {
    // there doesn't seem to be a function to unregister a server.
}

pub fn unregister_scene() {}
