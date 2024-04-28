use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::fluids::rapier_fluid_2d::RapierFluid2D;
use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::spaces::rapier_space_2d::RapierSpace2D;
use godot::builtin::Rid;
use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};

pub struct RapierPhysicsSingleton2D {
    pub shapes: HashMap<Rid, Box<dyn IRapierShape2D>>,
    pub spaces: HashMap<Rid, Box<RapierSpace2D>>,
    pub collision_objects: HashMap<Rid, Box<dyn IRapierCollisionObject2D>>,
    pub joints: HashMap<Rid, Box<dyn IRapierJoint2D>>,
    pub fluids: HashMap<Rid, Box<RapierFluid2D>>,
}
impl RapierPhysicsSingleton2D {
    pub fn new() -> RapierPhysicsSingleton2D {
        RapierPhysicsSingleton2D {
            shapes: HashMap::new(),
            spaces: HashMap::new(),
            collision_objects: HashMap::new(),
            joints: HashMap::new(),
            fluids: HashMap::new(),
        }
    }
}
unsafe impl Send for RapierPhysicsSingleton2D {}

pub fn physics_singleton() -> &'static Mutex<RapierPhysicsSingleton2D> {
    static HOLDER: OnceLock<Mutex<RapierPhysicsSingleton2D>> = OnceLock::new();
    HOLDER.get_or_init(|| Mutex::new(RapierPhysicsSingleton2D::new()))
}
