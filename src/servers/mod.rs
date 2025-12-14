use godot::classes::ProjectSettings;
use godot::prelude::*;

use crate::servers::rapier_project_settings::RapierProjectSettings;
pub mod rapier_math;
#[cfg(feature = "dim2")]
pub mod rapier_physics_server_2d;
#[cfg(feature = "dim3")]
pub mod rapier_physics_server_3d;
pub mod rapier_physics_server_extra;
pub mod rapier_physics_server_impl;
pub mod rapier_physics_singleton;
pub mod rapier_project_settings;
#[cfg(feature = "dim2")]
pub type RapierPhysicsServer = rapier_physics_server_2d::RapierPhysicsServer2D;
#[cfg(feature = "dim3")]
pub type RapierPhysicsServer = rapier_physics_server_3d::RapierPhysicsServer3D;
#[cfg(feature = "dim2")]
pub fn register_server() {
    use godot::classes::PhysicsServer2DManager;
    let mut manager = PhysicsServer2DManager::singleton();
    let factory =
        crate::servers::rapier_physics_server_2d::RapierPhysicsServerFactory2D::new_alloc();
    manager.register_server("Rapier2D", &factory.callable("create_server"));
}
#[cfg(feature = "dim3")]
pub fn register_server() {
    use godot::classes::PhysicsServer3DManager;
    let mut manager = PhysicsServer3DManager::singleton();
    let factory =
        crate::servers::rapier_physics_server_3d::RapierPhysicsServerFactory3D::new_alloc();
    manager.register_server("Rapier3D", &factory.callable("create_server"));
}
#[cfg(feature = "dim2")]
fn print_version() {
    let project_settings = ProjectSettings::singleton();
    let physics_engine: String = project_settings
        .get_setting("physics/2d/physics_engine")
        .try_to()
        .unwrap_or_default();
    if physics_engine != "Rapier2D" {
        godot_print_rich!(
            "[color=red]PHYSICS ENGINE 2D: {}[/color]. Go to [b]Advanced Settings -> Physics -> 2D[/b] to change it to [b]Rapier2D[/b].",
            physics_engine
        );
    } else {
        godot_print_rich!(
            "[color=green]PHYSICS ENGINE 2D: {} v0.8.25[/color]",
            physics_engine
        );
    }
}
#[cfg(feature = "dim3")]
fn print_version() {
    let project_settings = ProjectSettings::singleton();
    let physics_engine: String = project_settings
        .get_setting("physics/3d/physics_engine")
        .try_to()
        .unwrap_or_default();
    if physics_engine != "Rapier3D" {
        godot_print_rich!(
            "[color=red]PHYSICS ENGINE 3D: {}[/color]. Go to [b]Advanced Settings -> Physics -> 3D[/b] to change it to [b]Rapier3D[/b].",
            physics_engine
        );
    } else {
        godot_print_rich!(
            "[color=green]PHYSICS ENGINE 3D: {} v0.8.25[/color]",
            physics_engine
        );
    }
}
pub fn register_scene() {
    RapierProjectSettings::register_settings();
    print_version();
}
