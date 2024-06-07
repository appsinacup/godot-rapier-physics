use crate::bodies3d::rapier_collision_object_3d::IRapierCollisionObject3D;
use crate::fluids3d::rapier_fluid_3d::RapierFluid3D;
use crate::joints3d::rapier_joint_3d::IRapierJoint3D;
use crate::rapier3d::handle::Handle;
use crate::shapes3d::rapier_shape_3d::IRapierShape3D;
use crate::spaces3d::rapier_space_3d::RapierSpace3D;
use godot::builtin::Rid;
use std::collections::HashMap;

pub struct RapierShapesSingleton3D {
    pub shapes: HashMap<Rid, Box<dyn IRapierShape3D>>,
}
pub struct RapierActiveSpacesSingleton3D {
    pub active_spaces: HashMap<Handle, Rid>,
}
pub struct RapierSpacesSingleton3D {
    pub spaces: HashMap<Rid, Box<RapierSpace3D>>,
}
pub struct RapierBodiesSingleton3D {
    pub collision_objects: HashMap<Rid, Box<dyn IRapierCollisionObject3D>>,
}
pub struct RapierJointsSingleton3D {
    pub joints: HashMap<Rid, Box<dyn IRapierJoint3D>>,
}
pub struct RapierFluidsSingleton3D {
    pub fluids: HashMap<Rid, Box<RapierFluid3D>>,
}

impl RapierShapesSingleton3D {
    pub fn new() -> RapierShapesSingleton3D {
        RapierShapesSingleton3D {
            shapes: HashMap::new(),
        }
    }
}
impl RapierActiveSpacesSingleton3D {
    pub fn new() -> RapierActiveSpacesSingleton3D {
        RapierActiveSpacesSingleton3D {
            active_spaces: HashMap::new(),
        }
    }
}
impl RapierSpacesSingleton3D {
    pub fn new() -> RapierSpacesSingleton3D {
        RapierSpacesSingleton3D {
            spaces: HashMap::new(),
        }
    }
}
impl RapierBodiesSingleton3D {
    pub fn new() -> RapierBodiesSingleton3D {
        RapierBodiesSingleton3D {
            collision_objects: HashMap::new(),
        }
    }
}
impl RapierJointsSingleton3D {
    pub fn new() -> RapierJointsSingleton3D {
        RapierJointsSingleton3D {
            joints: HashMap::new(),
        }
    }
}
impl RapierFluidsSingleton3D {
    pub fn new() -> RapierFluidsSingleton3D {
        RapierFluidsSingleton3D {
            fluids: HashMap::new(),
        }
    }
}
unsafe impl Send for RapierShapesSingleton3D {}
unsafe impl Sync for RapierShapesSingleton3D {}
unsafe impl Send for RapierSpacesSingleton3D {}
unsafe impl Sync for RapierSpacesSingleton3D {}
unsafe impl Send for RapierActiveSpacesSingleton3D {}
unsafe impl Sync for RapierActiveSpacesSingleton3D {}
unsafe impl Send for RapierBodiesSingleton3D {}
unsafe impl Sync for RapierBodiesSingleton3D {}
unsafe impl Send for RapierJointsSingleton3D {}
unsafe impl Sync for RapierJointsSingleton3D {}
unsafe impl Send for RapierFluidsSingleton3D {}
unsafe impl Sync for RapierFluidsSingleton3D {}

pub fn shapes_singleton() -> &'static mut RapierShapesSingleton3D {
    static mut SINGLETON: Option<RapierShapesSingleton3D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierShapesSingleton3D {
                shapes: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}

pub fn active_spaces_singleton() -> &'static mut RapierActiveSpacesSingleton3D {
    static mut SINGLETON: Option<RapierActiveSpacesSingleton3D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierActiveSpacesSingleton3D {
                active_spaces: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn spaces_singleton() -> &'static mut RapierSpacesSingleton3D {
    static mut SINGLETON: Option<RapierSpacesSingleton3D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierSpacesSingleton3D {
                spaces: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn bodies_singleton() -> &'static mut RapierBodiesSingleton3D {
    static mut SINGLETON: Option<RapierBodiesSingleton3D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierBodiesSingleton3D {
                collision_objects: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn joints_singleton() -> &'static mut RapierJointsSingleton3D {
    static mut SINGLETON: Option<RapierJointsSingleton3D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierJointsSingleton3D {
                joints: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn fluids_singleton() -> &'static mut RapierFluidsSingleton3D {
    static mut SINGLETON: Option<RapierFluidsSingleton3D> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(RapierFluidsSingleton3D {
                fluids: HashMap::new(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
