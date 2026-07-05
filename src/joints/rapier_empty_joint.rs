use godot::classes::*;
use joints::rapier_joint::IRapierJoint;
use joints::rapier_joint::impl_rapier_joint_base;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;
use servers::rapier_physics_singleton::RapierId;

use super::rapier_joint_base::RapierJointBase;
use super::rapier_joint_base::RapierJointType;
use crate::rapier_wrapper::prelude::*;
use crate::*;
pub struct RapierEmptyJoint {
    base: RapierJointBase,
}
impl RapierEmptyJoint {
    pub fn new(id: RapierId) -> Self {
        Self {
            base: RapierJointBase::new(
                id,
                Rid::Invalid,
                RapierId::default(),
                WorldHandle::default(),
                JointHandle::default(),
                RapierJointType::Impulse,
            ),
        }
    }
}
impl_rapier_joint_base!(RapierEmptyJoint, JointType::MAX);
impl Drop for RapierJointBase {
    fn drop(&mut self) {
        if self.get_handle() != JointHandle::default() {
            godot_error!("RapierJointBase leaked");
        }
    }
}
