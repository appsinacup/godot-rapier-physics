use godot::{builtin::Rid, engine::physics_server_2d};

use crate::servers::rapier_physics_singleton_2d::physics_collision_objects_singleton_mutex_guard;


pub trait IRapierJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType;
    fn disable_collisions_between_bodies(&mut self, disabled: bool);
    fn copy_settings_from(&mut self, joint: RapierJointBase2D);
}

pub struct RapierJointBase2D {
    bias: f32,
    max_bias: f32,
    max_force: f32,
    rid: Rid,
    body_a: Rid,
    body_b: Rid,
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
        }
    }

    fn set_max_force(&mut self, force: f32) {
        self.max_force = force;
    }

    fn get_max_force(&self) -> f32 {
        self.max_force
    }

    fn set_bias(&mut self, bias: f32) {
        self.bias = bias;
    }

    fn get_bias(&self) -> f32 {
        self.bias
    }

    fn set_max_bias(&mut self, bias: f32) {
        self.max_bias = bias;
    }

    fn get_max_bias(&self) -> f32 {
        self.max_bias
    }

    fn get_rid(&self) -> Rid {
        self.rid
    }

    // Careful when doing this you must also update the place where it's stored.
    fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

    fn get_space(&self) -> Rid {
        let binding = physics_collision_objects_singleton_mutex_guard();
        if self.body_a.is_valid() {
            let body_a = binding.get(&self.body_a);
            if let Some(body_a) = body_a {
                return body_a.get_space();
            }
        }
        let body_b = binding.get(&self.body_b);
        
        let space_a = body_a.unwrap()
        physics_server_2d::get_space(self.body_a)
    }
}
