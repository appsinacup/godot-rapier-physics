use crate::servers3d::rapier_physics_server_3d::*;
use godot::{obj::NewAlloc, prelude::*};

#[derive(GodotClass, Default)]
#[class(base=Object,init,tool)]
pub struct RapierPhysicsServerFactory3D;

#[godot_api]
impl RapierPhysicsServerFactory3D {
    #[func]
    fn create_server() -> Gd<RapierPhysicsServer3D> {
        RapierPhysicsServer3D::new_alloc()
    }
}
