use std::collections::HashMap;

use godot::prelude::*;

use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::fluids::rapier_fluid::RapierFluid;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::spaces::rapier_space::RapierSpace;
pub struct RapierShapesSingleton {
    pub shapes: HashMap<Rid, Box<dyn IRapierShape>>,
}
pub struct RapierActiveSpacesSingleton {
    pub active_spaces: HashMap<Handle, Rid>,
}
pub struct RapierSpacesSingleton {
    pub spaces: HashMap<Rid, Box<RapierSpace>>,
}
pub struct RapierBodiesSingleton {
    pub collision_objects: HashMap<Rid, Box<dyn IRapierCollisionObject>>,
}
pub struct RapierJointsSingleton {
    pub joints: HashMap<Rid, Box<dyn IRapierJoint>>,
}
pub struct RapierFluidsSingleton {
    pub fluids: HashMap<Rid, Box<RapierFluid>>,
}
unsafe impl Send for RapierShapesSingleton {}
unsafe impl Sync for RapierShapesSingleton {}
unsafe impl Send for RapierSpacesSingleton {}
unsafe impl Sync for RapierSpacesSingleton {}
unsafe impl Send for RapierActiveSpacesSingleton {}
unsafe impl Sync for RapierActiveSpacesSingleton {}
unsafe impl Send for RapierBodiesSingleton {}
unsafe impl Sync for RapierBodiesSingleton {}
unsafe impl Send for RapierJointsSingleton {}
unsafe impl Sync for RapierJointsSingleton {}
unsafe impl Send for RapierFluidsSingleton {}
unsafe impl Sync for RapierFluidsSingleton {}
pub fn shapes_singleton() -> &'static mut RapierShapesSingleton {
    static mut SINGLETON: Option<RapierShapesSingleton> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierShapesSingleton {
                shapes: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn active_spaces_singleton() -> &'static mut RapierActiveSpacesSingleton {
    static mut SINGLETON: Option<RapierActiveSpacesSingleton> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierActiveSpacesSingleton {
                active_spaces: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn spaces_singleton() -> &'static mut RapierSpacesSingleton {
    static mut SINGLETON: Option<RapierSpacesSingleton> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierSpacesSingleton {
                spaces: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn bodies_singleton() -> &'static mut RapierBodiesSingleton {
    static mut SINGLETON: Option<RapierBodiesSingleton> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierBodiesSingleton {
                collision_objects: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn joints_singleton() -> &'static mut RapierJointsSingleton {
    static mut SINGLETON: Option<RapierJointsSingleton> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierJointsSingleton {
                joints: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn fluids_singleton() -> &'static mut RapierFluidsSingleton {
    static mut SINGLETON: Option<RapierFluidsSingleton> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierFluidsSingleton {
                fluids: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
