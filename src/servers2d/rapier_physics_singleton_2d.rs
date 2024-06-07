use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::fluids::rapier_fluid_2d::RapierFluid2D;
use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::rapier_wrapper::handle::Handle;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::spaces::rapier_space_2d::RapierSpace2D;
use godot::builtin::Rid;
use std::collections::HashMap;

pub struct RapierShapesSingleton2D {
    pub shapes: HashMap<Rid, Box<dyn IRapierShape2D>>,
}
pub struct RapierActiveSpacesSingleton2D {
    pub active_spaces: HashMap<Handle, Rid>,
}
pub struct RapierSpacesSingleton2D {
    pub spaces: HashMap<Rid, Box<RapierSpace2D>>,
}
pub struct RapierBodiesSingleton2D {
    pub collision_objects: HashMap<Rid, Box<dyn IRapierCollisionObject2D>>,
}
pub struct RapierJointsSingleton2D {
    pub joints: HashMap<Rid, Box<dyn IRapierJoint2D>>,
}
pub struct RapierFluidsSingleton2D {
    pub fluids: HashMap<Rid, Box<RapierFluid2D>>,
}

impl RapierShapesSingleton2D {
    pub fn new() -> RapierShapesSingleton2D {
        RapierShapesSingleton2D {
            shapes: HashMap::new(),
        }
    }
}
impl RapierActiveSpacesSingleton2D {
    pub fn new() -> RapierActiveSpacesSingleton2D {
        RapierActiveSpacesSingleton2D {
            active_spaces: HashMap::new(),
        }
    }
}
impl RapierSpacesSingleton2D {
    pub fn new() -> RapierSpacesSingleton2D {
        RapierSpacesSingleton2D {
            spaces: HashMap::new(),
        }
    }
}
impl RapierBodiesSingleton2D {
    pub fn new() -> RapierBodiesSingleton2D {
        RapierBodiesSingleton2D {
            collision_objects: HashMap::new(),
        }
    }
}
impl RapierJointsSingleton2D {
    pub fn new() -> RapierJointsSingleton2D {
        RapierJointsSingleton2D {
            joints: HashMap::new(),
        }
    }
}
impl RapierFluidsSingleton2D {
    pub fn new() -> RapierFluidsSingleton2D {
        RapierFluidsSingleton2D {
            fluids: HashMap::new(),
        }
    }
}
unsafe impl Send for RapierShapesSingleton2D {}
unsafe impl Sync for RapierShapesSingleton2D {}

unsafe impl Send for RapierSpacesSingleton2D {}
unsafe impl Sync for RapierSpacesSingleton2D {}
unsafe impl Send for RapierActiveSpacesSingleton2D {}
unsafe impl Sync for RapierActiveSpacesSingleton2D {}
unsafe impl Send for RapierBodiesSingleton2D {}
unsafe impl Sync for RapierBodiesSingleton2D {}
unsafe impl Send for RapierJointsSingleton2D {}
unsafe impl Sync for RapierJointsSingleton2D {}
unsafe impl Send for RapierFluidsSingleton2D {}
unsafe impl Sync for RapierFluidsSingleton2D {}

pub fn shapes_singleton() -> &'static mut RapierShapesSingleton2D {
    static mut SINGLETON: Option<RapierShapesSingleton2D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierShapesSingleton2D {
                shapes: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}

pub fn active_spaces_singleton() -> &'static mut RapierActiveSpacesSingleton2D {
    static mut SINGLETON: Option<RapierActiveSpacesSingleton2D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierActiveSpacesSingleton2D {
                active_spaces: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn spaces_singleton() -> &'static mut RapierSpacesSingleton2D {
    static mut SINGLETON: Option<RapierSpacesSingleton2D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierSpacesSingleton2D {
                spaces: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn bodies_singleton() -> &'static mut RapierBodiesSingleton2D {
    static mut SINGLETON: Option<RapierBodiesSingleton2D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierBodiesSingleton2D {
                collision_objects: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn joints_singleton() -> &'static mut RapierJointsSingleton2D {
    static mut SINGLETON: Option<RapierJointsSingleton2D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierJointsSingleton2D {
                joints: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn fluids_singleton() -> &'static mut RapierFluidsSingleton2D {
    static mut SINGLETON: Option<RapierFluidsSingleton2D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierFluidsSingleton2D {
                fluids: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
