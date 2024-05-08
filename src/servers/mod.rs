use godot::{engine::PhysicsServer2DManager, prelude::*};

use crate::servers::rapier_physics_server_factory_2d::*;
use crate::servers::rapier_project_settings::RapierProjectSettings;

pub mod rapier_physics_server_2d;
pub mod rapier_physics_server_factory_2d;
pub mod rapier_physics_singleton_2d;
pub mod rapier_project_settings;

pub fn register_server() {
    let mut manager = PhysicsServer2DManager::singleton();
    let factory = RapierPhysicsServerFactory2D::new_alloc();
    manager.register_server("Rapier2D".into(), factory.callable("create_server"));
}

pub fn register_scene() {
    RapierProjectSettings::register_settings();
}

pub fn unregister_server() {
    // TODO
}

pub fn unregister_scene() {
    // TODO
}
