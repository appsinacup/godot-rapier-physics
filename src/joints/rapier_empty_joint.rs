use godot::classes::*;
use joints::rapier_joint::IRapierJoint;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;

use super::rapier_joint_base::RapierJointBase;
use crate::rapier_wrapper::prelude::*;
use crate::*;
pub struct RapierEmptyJoint {
    base: RapierJointBase,
}
impl Default for RapierEmptyJoint {
    fn default() -> Self {
        Self::new()
    }
}
impl RapierEmptyJoint {
    pub fn new() -> Self {
        Self {
            base: RapierJointBase::new(WorldHandle::default(), JointHandle::default()),
        }
    }
}
impl IRapierJoint for RapierEmptyJoint {
    fn get_type(&self) -> JointType {
        JointType::MAX
    }

    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }
}
impl Drop for RapierJointBase {
    fn drop(&mut self) {
        if self.get_handle() != JointHandle::default() {
            godot_error!("RapierJointBase leaked");
        }
    }
}
