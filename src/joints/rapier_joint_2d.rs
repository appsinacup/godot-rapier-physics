use crate::{
    bodies::rapier_collision_object_2d::IRapierCollisionObject2D,
    rapier2d::{
        handle::{invalid_handle, Handle},
        joint::joint_change_disable_collision,
    },
    servers::rapier_physics_singleton_2d::physics_singleton,
};
use godot::{builtin::Rid, engine::physics_server_2d};

pub trait IRapierJoint2D {
    fn get_base(&self) -> &RapierJointBase2D;
    fn get_type(&self) -> physics_server_2d::JointType;
}

pub struct RapierJointBase2D {
    bias: f32,
    max_bias: f32,
    max_force: f32,
    rid: Rid,
    body_a: Rid,
    body_b: Rid,
    handle: Handle,
    disabled_collisions_between_bodies: bool,
}

impl RapierJointBase2D {
    pub fn new(rid: Rid, body_a: Rid, body_b: Rid) -> Self {
        Self {
            bias: 0.0,
            max_bias: 3.40282e38,
            max_force: 3.40282e38,
            rid: rid,
            body_a: body_a,
            body_b: body_b,
            handle: invalid_handle(),
            disabled_collisions_between_bodies: false,
        }
    }

    pub fn get_handle(&self) -> Handle {
        self.handle
    }

    pub fn set_handle(&mut self, handle: Handle) {
        self.handle = handle
    }

    pub fn set_max_force(&mut self, force: f32) {
        self.max_force = force;
    }

    pub fn get_max_force(&self) -> f32 {
        self.max_force
    }

    pub fn set_bias(&mut self, bias: f32) {
        self.bias = bias;
    }

    pub fn get_bias(&self) -> f32 {
        self.bias
    }

    pub fn set_max_bias(&mut self, bias: f32) {
        self.max_bias = bias;
    }

    pub fn get_max_bias(&self) -> f32 {
        self.max_bias
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    // Careful when doing this you must also update the place where it's stored.
    pub fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

    pub fn get_space(&self) -> Rid {
        let lock = physics_singleton().lock().unwrap();
        if self.body_a.is_valid() {
            let body_a: Option<&Box<dyn IRapierCollisionObject2D>> =
                lock.collision_objects.get(&self.body_a);
            if let Some(body_a) = body_a {
                //return body_a.get_space();
            }
        }
        let body_b = lock.collision_objects.get(&self.body_b);
        return Rid::Invalid;
        //let space_a = body_a.unwrap()
        //physics_server_2d::get_space(self.body_a)
    }

    pub fn disable_collisions_between_bodies(&mut self, disabled: bool) {
        self.disabled_collisions_between_bodies = disabled;
        if self.handle.is_valid() {
            //joint_change_disable_collision(world_handle, joint_handle, disable_collision)
            // Joint not yet created, when it will be created it will have disable collision flag set
            //rapier2d::geometry::ColliderSet::joint_change_disable_collision(
            //    &mut self.space_handle,
            //    &mut self.handle,
            //    self.disabled_collisions_between_bodies,
            //);
        }
    }

    pub fn is_disabled_collisions_between_bodies(&self) -> bool {
        self.disabled_collisions_between_bodies
    }

    pub fn copy_settings_from(&mut self, joint: RapierJointBase2D) {
        self.set_rid(joint.get_rid());
        self.set_max_force(joint.get_max_force());
        self.set_bias(joint.get_bias());
        self.set_max_bias(joint.get_max_bias());
        self.disable_collisions_between_bodies(joint.is_disabled_collisions_between_bodies());
    }
}
