use std::{collections::HashMap, sync::{Mutex, OnceLock}};

use godot::builtin::Rid;

use crate::joints::rapier_joint_2d::IRapierJoint2D;


pub struct RapierPhysicsSingleton2D {
    //shape_owner: HashMap<Rid, RapierShape2D>,
    //space_owner: HashMap<Rid, RapierSpace2D>,
    //area_owner: HashMap<Rid, RapierArea2D>,
    //body_owner: HashMap<Rid, RapierBody2D>,
    joints: HashMap<Rid, Box<dyn IRapierJoint2D>>,
    //fluid_owner: HashMap<Rid, RapierFluid2D>,
}


impl RapierPhysicsSingleton2D {
    pub fn new() -> Self {
        Self {
            //shape_owner: HashMap::new(),
            //space_owner: HashMap::new(),
            //area_owner: HashMap::new(),
            //body_owner: HashMap::new(),
            joints: HashMap::new(),
            //fluid_owner: HashMap::new(),
        }
    }
}


//pub fn singleton() -> &'static Mutex<RapierPhysicsSingleton2D> {
//    static HOLDER: OnceLock<Mutex<RapierPhysicsSingleton2D>> = OnceLock::new();
//    HOLDER.get_or_init(|| Mutex::new(RapierPhysicsSingleton2D::new()))
//}