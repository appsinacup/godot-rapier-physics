use godot::{engine::PhysicsServer2DManager, prelude::*};

use crate::servers2d::rapier_physics_server_factory_2d::*;
use crate::servers2d::rapier_project_settings_2d::RapierProjectSettings2D;

pub mod rapier_physics_server_2d;
pub mod rapier_physics_server_factory_2d;
pub mod rapier_physics_singleton_2d;
pub mod rapier_project_settings_2d;

pub fn register_server() {
    let mut manager = PhysicsServer2DManager::singleton();
    let factory = RapierPhysicsServerFactory2D::new_alloc();
    manager.register_server("Rapier2D".into(), factory.callable("create_server"));
}

pub fn register_scene() {
    RapierProjectSettings2D::register_settings();
}

pub fn unregister_server() {
    // there doesn't seem to be a function to unregister a server.
}

pub fn unregister_scene() {}
