use crate::bodies::rapier_area_2d::RapierArea2D;
use crate::bodies::rapier_body_2d::RapierBody2D;
use crate::bodies::rapier_collision_object_2d::{CollisionObjectType, IRapierCollisionObject2D};
use crate::fluids::rapier_fluid_2d::RapierFluid2D;
use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::rapier2d::handle::Handle;
use crate::shapes::rapier_circle_shape_2d::RapierCircleShape2D;
use crate::shapes::rapier_shape_2d::{IRapierShape2D};
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
    pub active_spaces: HashMap<Handle, Rid>,
}
impl RapierPhysicsSingleton2D {
    pub fn new() -> RapierPhysicsSingleton2D {
        RapierPhysicsSingleton2D {
            shapes: HashMap::new(),
            spaces: HashMap::new(),
            active_spaces: HashMap::new(),
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

pub fn get_shape(rid: Rid) -> &'static IRapierShape2D {
    let lock = physics_singleton().lock().unwrap();
    let shape = lock.shapes.get(&rid);
    if let Some(shape) = shape {
        return shape;
    }
    return &RapierCircleShape2D::new(Rid::Invalid);
}

pub fn get_space(rid: Rid) -> &'static RapierSpace2D {
    let lock = physics_singleton().lock().unwrap();
    let space = lock.spaces.get(&rid);
    if let Some(space) = space {
        return space;
    }
    return &RapierSpace2D::new(Rid::Invalid);
}

pub fn get_collision_object(rid: Rid) -> &'static IRapierCollisionObject2D {
    let lock = physics_singleton().lock().unwrap();
    let collision_object = lock.collision_objects.get(&rid);
    if let Some(collision_object) = collision_object {
        return collision_object;
    }
    return &RapierBody2D::new(Rid::Invalid);
}

pub fn get_body(rid: Rid) -> &'static RapierBody2D {
    let body = get_collision_object(rid);
    if let Some(body) = body {
        if body.get_base().get_type() == CollisionObjectType::Body {
            return body as &RapierBody2D;
        }
    }
    return &RapierBody2D::new(Rid::Invalid);
}

pub fn get_area(rid: Rid) -> &'static RapierArea2D {
    let area = get_collision_object(rid);
    if let Some(area) = area {
        if area.get_base().get_type() == CollisionObjectType::Area {
            return area as &RapierArea2D;
        }
    }
    return &RapierArea2D::new(Rid::Invalid);
}

pub fn get_joint(rid: Rid) -> &'static IRapierJoint2D {
    let lock = physics_singleton().lock().unwrap();
    let joint = lock.joints.get(&rid);
    if let Some(joint) = joint {
        return joint;
    }
    return &RapierBody2D::new(Rid::Invalid);
}

pub fn get_fluid(rid: Rid) -> &'static RapierFluid2D {
    let lock = physics_singleton().lock().unwrap();
    let fluid = lock.fluids.get(&rid);
    if let Some(fluid) = fluid {
        return fluid;
    }
    return &RapierFluid2D::new(Rid::Invalid);
}
