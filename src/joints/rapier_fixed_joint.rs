use godot::classes::*;
use godot::prelude::*;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;

use super::rapier_joint_base::RapierJointBase;
use super::rapier_joint_base::RapierJointType;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::joints::rapier_joint::impl_rapier_joint_base;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::world_to_local_no_scale;
use crate::types::*;
/// Locks all relative motion between two bodies.
pub struct RapierFixedJoint {
    base: RapierJointBase,
}
impl RapierFixedJoint {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: RapierId,
        rid: Rid,
        p_anchor_a: Vector,
        p_anchor_b: Vector,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        joint_type: RapierJointType,
    ) -> Self {
        let invalid_joint = Self {
            base: RapierJointBase::default(),
        };
        if body_a.get_base().get_rid() == body_b.get_base().get_rid() {
            return invalid_joint;
        }
        if !body_a.get_base().is_valid()
            || !body_b.get_base().is_valid()
            || body_a.get_base().get_space_id() != body_b.get_base().get_space_id()
        {
            return invalid_joint;
        }
        let anchor_a = world_to_local_no_scale(&body_a.get_base().get_transform(), p_anchor_a);
        let anchor_b = world_to_local_no_scale(&body_b.get_base().get_transform(), p_anchor_b);
        let space_handle = body_a.get_base().get_space_id();
        let space_id = body_a.get_base().get_space_id();
        let handle = physics_engine.joint_create_fixed(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            vector_to_rapier(anchor_a),
            vector_to_rapier(anchor_b),
            joint_type,
            true,
        );
        Self {
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle, joint_type),
        }
    }
}
impl_rapier_joint_base!(RapierFixedJoint, JointType::MAX);
