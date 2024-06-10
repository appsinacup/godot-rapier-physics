use godot::obj::NewAlloc;
use godot::prelude::*;

use crate::servers::rapier_physics_server_3d::*;
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
