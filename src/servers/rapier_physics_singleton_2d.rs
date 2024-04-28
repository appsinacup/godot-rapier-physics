use crate::bodies::rapier_area_2d::RapierArea2D;
use crate::bodies::rapier_body_2d::RapierBody2D;
use crate::fluids::rapier_fluid_2d::RapierFluid2D;
use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::spaces::rapier_space_2d::RapierSpace2D;
use godot::builtin::Rid;
use std::collections::HashMap;
use std::sync::{Mutex, MutexGuard, OnceLock};

pub struct RapierPhysicsSingleton2D {
    pub shapes: HashMap<Rid, Box<dyn IRapierShape2D>>,
    pub spaces: HashMap<Rid, RapierSpace2D>,
    pub areas: HashMap<Rid, Box<RapierArea2D>>,
    pub bodies: HashMap<Rid, Box<RapierBody2D>>,
    pub joints: HashMap<Rid, Box<dyn IRapierJoint2D>>,
    pub fluids: HashMap<Rid, Box<RapierFluid2D>>,
}

impl RapierPhysicsSingleton2D {
    pub fn new() -> RapierPhysicsSingleton2D {
        RapierPhysicsSingleton2D {
            shapes: HashMap::new(),
            spaces: HashMap::new(),
            areas: HashMap::new(),
            bodies: HashMap::new(),
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

pub fn physics_singleton_mutex_guard() -> MutexGuard<'static, RapierPhysicsSingleton2D> {
    physics_singleton().lock().unwrap()
}
