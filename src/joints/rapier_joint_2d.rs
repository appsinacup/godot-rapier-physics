use crate::rapier2d::handle::{invalid_handle, Handle};
use godot::{builtin::Rid, engine::physics_server_2d};

pub trait IRapierJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType;
}

pub struct RapierJointBase2D {
    bias: f32,
    max_bias: f32,
    max_force: f32,
    disabled_collisions_between_bodies: bool,
    rid: Rid,
    // body A
    body_a_rid: Rid,
    // body B
    body_b_rid: Option<Rid>,
    space_rid: Rid,
    handle: Handle,
}

impl RapierJointBase2D {
    pub fn new(body_a_rid: Rid, body_b_rid: Option<Rid>) -> Self {
        Self {
            bias: 0.0,
            max_bias: 3.40282e38,
            max_force: 3.40282e38,
            disabled_collisions_between_bodies: true,
            rid: Rid::Invalid,
            body_a_rid,
            body_b_rid,
            space_rid: Rid::Invalid,
            handle: invalid_handle(),
        }
    }

    pub fn disable_collisions_between_bodies(&mut self, disabled: bool) {
        self.disabled_collisions_between_bodies = disabled;
        if self.handle.is_valid() {
            // Joint not yet created, when it will be created it will have disable collision flag set
            //rapier2d::geometry::ColliderSet::joint_change_disable_collision(
            //    &mut self.space_handle,
            //    &mut self.handle,
            //    self.disabled_collisions_between_bodies,
            //);
        }
    }

    fn is_disabled_collisions_between_bodies(&self) -> bool {
        self.disabled_collisions_between_bodies
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

    fn copy_settings_from(&mut self, joint: RapierJointBase2D) {
        self.set_rid(joint.get_rid());
        self.set_max_force(joint.get_max_force());
        self.set_bias(joint.get_bias());
        self.set_max_bias(joint.get_max_bias());
        self.disable_collisions_between_bodies(joint.is_disabled_collisions_between_bodies());
    }

    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::MAX
    }

    fn get_rid(&self) -> Rid {
        self.rid
    }

    fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }
}

impl Drop for RapierJointBase2D {
    fn drop(&mut self) {
        if self.handle.is_valid() {
            // ERR_FAIL_COND(!rapier2d::geometry::ColliderSet::is_valid(&self.space_handle));
            //rapier2d::geometry::ColliderSet::joint_destroy(&mut self.space_handle, &mut self.handle);
        }
    }
}
