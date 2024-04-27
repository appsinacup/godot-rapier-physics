use crate::joints::rapier_joint_2d::IRapierJoint2D;
use godot::{builtin::Rid, engine::physics_server_2d};

use super::rapier_joint_2d::RapierJointBase2D;
pub struct RapierGrooveJoint2D {
    base: RapierJointBase2D,
}

impl RapierGrooveJoint2D {
    pub fn new(body_a_rid: Rid, body_b_rid: Option<Rid>) -> Self {
        Self {
            base: RapierJointBase2D::new(body_a_rid, body_b_rid),
        }
    }
}

impl IRapierJoint2D for RapierGrooveJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::GROOVE
    }
}
