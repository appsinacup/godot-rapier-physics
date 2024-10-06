use std::str::FromStr;

use godot::classes::ConfigFile;
use godot::classes::ProjectSettings;
use godot::global;
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
    manager.register_server("Rapier2D".into(), factory.callable("create_server"));
}
#[cfg(feature = "dim3")]
pub fn register_server() {
    use godot::classes::PhysicsServer3DManager;
    let mut manager = PhysicsServer3DManager::singleton();
    let factory =
        crate::servers::rapier_physics_server_3d::RapierPhysicsServerFactory3D::new_alloc();
    manager.register_server("Rapier3D".into(), factory.callable("create_server"));
}
#[cfg(feature = "dim2")]
fn print_version() {
    let mut config_file = ConfigFile::new_gd();
    let err = config_file
        .load(GString::from_str("res://addons/godot-rapier2d/plugin.info.cfg").unwrap_or_default());
    if err != global::Error::OK {
        godot_error!("Error loading plugin.info.cfg: {:?}", err);
        return;
    }
    let version = config_file.get_value(
        GString::from_str("plugin").unwrap_or_default(),
        GString::from_str("version").unwrap_or_default(),
    );
    let project_settings = ProjectSettings::singleton();
    let physics_engine: String = project_settings
        .get_setting(GString::from_str("physics/2d/physics_engine").unwrap_or_default())
        .try_to()
        .unwrap_or_default();
    if physics_engine != "Rapier2D" {
        godot_print_rich!(
            "[color=red]Physics Engine 2D: {}[/color]. Go to [b]Advanced Settings -> Physics -> 2D[/b] to change it to [b]Rapier2D[/b].",
            physics_engine
        );
    } else {
        godot_print_rich!(
            "[color=green]The current Physics Engine 2D: {} v{}[/color]",
            physics_engine,
            version
        );
    }
}
#[cfg(feature = "dim3")]
fn print_version() {
    let mut config_file = ConfigFile::new_gd();
    let err = config_file
        .load(GString::from_str("res://addons/godot-rapier3d/plugin.info.cfg").unwrap_or_default());
    if err != global::Error::OK {
        godot_error!("Error loading plugin.info.cfg: {:?}", err);
        return;
    }
    let version = config_file.get_value(
        GString::from_str("plugin").unwrap_or_default(),
        GString::from_str("version").unwrap_or_default(),
    );
    let project_settings = ProjectSettings::singleton();
    let physics_engine: String = project_settings
        .get_setting(GString::from_str("physics/3d/physics_engine").unwrap_or_default())
        .try_to()
        .unwrap_or_default();
    if physics_engine != "Rapier3D" {
        godot_print_rich!(
            "[color=red]Physics Engine 3D: {}[/color]. Go to [b]Advanced Settings -> Physics -> 3D[/b] to change it to [b]Rapier3D[/b].",
            physics_engine
        );
    } else {
        godot_print_rich!(
            "[color=green]The current Physics Engine 3D: {} v{}[/color]",
            physics_engine,
            version
        );
    }
}
pub fn register_scene() {
    print_version();
    RapierProjectSettings::register_settings();
}
