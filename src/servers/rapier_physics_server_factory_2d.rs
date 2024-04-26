use godot::{obj::NewAlloc, prelude::*};
use crate::servers::rapier_physics_server_2d::*;

#[derive(GodotClass)]
#[class(base=Object,init)]
pub struct RapierPhysicsServerFactory2D;

#[godot_api]
impl RapierPhysicsServerFactory2D {
    #[func]
    fn create_server() -> Gd<RapierPhysicsServer2D> {
        godot_print!("create_server");
        RapierPhysicsServer2D::new_alloc()
    }
}