use crate::joints::rapier_joint_2d::IRapierJoint2D;
use godot::{builtin::Rid, engine::physics_server_2d};

use super::rapier_joint_2d::RapierJointBase2D;
pub struct RapierGrooveJoint2D {
    base: RapierJointBase2D,
}

impl RapierGrooveJoint2D {
    pub fn new(rid: Rid, body_a: Rid, body_b: Rid) -> Self {
        Self {
            base: RapierJointBase2D::new(rid, body_a, body_b),
        }
    }
}

impl IRapierJoint2D for RapierGrooveJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::GROOVE
    }

    fn get_base(&self) -> &RapierJointBase2D {
        &self.base
    }
}
