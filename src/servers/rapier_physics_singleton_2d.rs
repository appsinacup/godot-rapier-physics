use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::fluids::rapier_fluid_2d::RapierFluid2D;
use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::spaces::rapier_space_2d::RapierSpace2D;
use godot::builtin::Rid;
use std::collections::HashMap;
use std::sync::{Mutex, MutexGuard, OnceLock};

pub struct RapierShapesSingleton2D {
    pub shapes: HashMap<Rid, Box<dyn IRapierShape2D>>,
}
impl RapierShapesSingleton2D {
    pub fn new() -> RapierShapesSingleton2D {
        RapierShapesSingleton2D {
            shapes: HashMap::new(),
        }
    }
    pub fn get(&self, rid: &Rid) -> Option<&Box<dyn IRapierShape2D>> {
        self.shapes.get(rid)
    }
    pub fn get_mut(&mut self, rid: &Rid) -> Option<&mut Box<dyn IRapierShape2D>> {
        self.shapes.get_mut(rid)
    }
    pub fn insert(&mut self, rid: Rid, object: Box<dyn IRapierShape2D>) {
        self.shapes.insert(rid, object);
    }
    pub fn remove(&mut self, rid: &Rid) -> Option<Box<dyn IRapierShape2D>> {
        self.shapes.remove(rid)
    }
}
unsafe impl Send for RapierShapesSingleton2D {}

pub struct RapierSpacesSingleton2D {
    pub spaces: HashMap<Rid, Box<RapierSpace2D>>,
}
impl RapierSpacesSingleton2D {
    pub fn new() -> RapierSpacesSingleton2D {
        RapierSpacesSingleton2D {
            spaces: HashMap::new(),
        }
    }
    pub fn get(&self, rid: &Rid) -> Option<&Box<RapierSpace2D>> {
        self.spaces.get(rid)
    }
    pub fn get_mut(&mut self, rid: &Rid) -> Option<&mut Box<RapierSpace2D>> {
        self.spaces.get_mut(rid)
    }
    pub fn insert(&mut self, rid: Rid, object: Box<RapierSpace2D>) {
        self.spaces.insert(rid, object);
    }
    pub fn remove(&mut self, rid: &Rid) -> Option<Box<RapierSpace2D>> {
        self.spaces.remove(rid)
    }
}
unsafe impl Send for RapierSpacesSingleton2D {}
pub struct RapierCollisionObjectsSingleton2D {
    pub collision_objects: HashMap<Rid, Box<dyn IRapierCollisionObject2D>>,
}
impl RapierCollisionObjectsSingleton2D {
    pub fn new() -> RapierCollisionObjectsSingleton2D {
        RapierCollisionObjectsSingleton2D {
            collision_objects: HashMap::new(),
        }
    }
    pub fn get(&self, rid: &Rid) -> Option<&Box<dyn IRapierCollisionObject2D>> {
        self.collision_objects.get(rid)
    }
    pub fn get_mut(&mut self, rid: &Rid) -> Option<&mut Box<dyn IRapierCollisionObject2D>> {
        self.collision_objects.get_mut(rid)
    }
    pub fn insert(&mut self, rid: Rid, object: Box<dyn IRapierCollisionObject2D>) {
        self.collision_objects.insert(rid, object);
    }
    pub fn remove(&mut self, rid: &Rid) -> Option<Box<dyn IRapierCollisionObject2D>> {
        self.collision_objects.remove(rid)
    }
}
unsafe impl Send for RapierCollisionObjectsSingleton2D {}
pub struct RapierJointsSingleton2D {
    pub joints: HashMap<Rid, Box<dyn IRapierJoint2D>>,
}
impl RapierJointsSingleton2D {
    pub fn new() -> RapierJointsSingleton2D {
        RapierJointsSingleton2D {
            joints: HashMap::new(),
        }
    }
    pub fn get(&self, rid: &Rid) -> Option<&Box<dyn IRapierJoint2D>> {
        self.joints.get(rid)
    }
    pub fn get_mut(&mut self, rid: &Rid) -> Option<&mut Box<dyn IRapierJoint2D>> {
        self.joints.get_mut(rid)
    }
    pub fn insert(&mut self, rid: Rid, object: Box<dyn IRapierJoint2D>) {
        self.joints.insert(rid, object);
    }
    pub fn remove(&mut self, rid: &Rid) -> Option<Box<dyn IRapierJoint2D>> {
        self.joints.remove(rid)
    }
}
unsafe impl Send for RapierJointsSingleton2D {}
pub struct RapierFluidsSingleton2D {
    pub fluids: HashMap<Rid, Box<RapierFluid2D>>,
}
impl RapierFluidsSingleton2D {
    pub fn new() -> RapierFluidsSingleton2D {
        RapierFluidsSingleton2D {
            fluids: HashMap::new(),
        }
    }
    pub fn get(&self, rid: &Rid) -> Option<&Box<RapierFluid2D>> {
        self.fluids.get(rid)
    }
    pub fn get_mut(&mut self, rid: &Rid) -> Option<&mut Box<RapierFluid2D>> {
        self.fluids.get_mut(rid)
    }
    pub fn insert(&mut self, rid: Rid, object: Box<RapierFluid2D>) {
        self.fluids.insert(rid, object);
    }
    pub fn remove(&mut self, rid: &Rid) -> Option<Box<RapierFluid2D>> {
        self.fluids.remove(rid)
    }
}

pub fn physics_shapes_singleton() -> &'static Mutex<RapierShapesSingleton2D> {
    static HOLDER: OnceLock<Mutex<RapierShapesSingleton2D>> = OnceLock::new();
    HOLDER.get_or_init(|| Mutex::new(RapierShapesSingleton2D::new()))
}

pub fn physics_shapes_singleton_mutex_guard() -> MutexGuard<'static, RapierShapesSingleton2D> {
    physics_shapes_singleton().lock().unwrap()
}

pub fn physics_spaces_singleton() -> &'static Mutex<RapierSpacesSingleton2D> {
    static HOLDER: OnceLock<Mutex<RapierSpacesSingleton2D>> = OnceLock::new();
    HOLDER.get_or_init(|| Mutex::new(RapierSpacesSingleton2D::new()))
}

pub fn physics_spaces_singleton_mutex_guard() -> MutexGuard<'static, RapierSpacesSingleton2D> {
    physics_spaces_singleton().lock().unwrap()
}

pub fn physics_collision_objects_singleton() -> &'static Mutex<RapierCollisionObjectsSingleton2D> {
    static HOLDER: OnceLock<Mutex<RapierCollisionObjectsSingleton2D>> = OnceLock::new();
    HOLDER.get_or_init(|| Mutex::new(RapierCollisionObjectsSingleton2D::new()))
}

pub fn physics_collision_objects_singleton_mutex_guard(
) -> MutexGuard<'static, RapierCollisionObjectsSingleton2D> {
    physics_collision_objects_singleton().lock().unwrap()
}

pub fn physics_joints_singleton() -> &'static Mutex<RapierJointsSingleton2D> {
    static HOLDER: OnceLock<Mutex<RapierJointsSingleton2D>> = OnceLock::new();
    HOLDER.get_or_init(|| Mutex::new(RapierJointsSingleton2D::new()))
}

pub fn physics_joints_singleton_mutex_guard() -> MutexGuard<'static, RapierJointsSingleton2D> {
    physics_joints_singleton().lock().unwrap()
}

pub fn physics_fluids_singleton() -> &'static Mutex<RapierFluidsSingleton2D> {
    static HOLDER: OnceLock<Mutex<RapierFluidsSingleton2D>> = OnceLock::new();
    HOLDER.get_or_init(|| Mutex::new(RapierFluidsSingleton2D::new()))
}

pub fn physics_fluids_singleton_mutex_guard() -> MutexGuard<'static, RapierFluidsSingleton2D> {
    physics_fluids_singleton().lock().unwrap()
}
