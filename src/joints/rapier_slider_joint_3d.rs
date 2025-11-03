use godot::classes::*;
use godot::prelude::*;

use super::rapier_joint_base::RapierJointBase;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::*;
pub struct RapierSliderJoint3D {
    base: RapierJointBase,
    linear_limit_lower: f32,
    linear_limit_upper: f32,
}
impl RapierSliderJoint3D {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: RapierId,
        rid: Rid,
        anchor_a: Vector3,
        anchor_b: Vector3,
        axis_a: Basis,
        axis_b: Basis,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            base: RapierJointBase::default(),
            linear_limit_lower: 0.0,
            linear_limit_upper: 0.0,
        };
        let body_a_rid = body_a.get_base().get_rid();
        let body_b_rid = body_b.get_base().get_rid();
        if body_a_rid == body_b_rid {
            return invalid_joint;
        }
        if !body_a.get_base().is_valid()
            || !body_b.get_base().is_valid()
            || body_a.get_base().get_space_id() != body_b.get_base().get_space_id()
        {
            return invalid_joint;
        }
        let rapier_anchor_a = vector_to_rapier(anchor_a);
        let rapier_anchor_b = vector_to_rapier(anchor_b);
        let rapier_axis_a = basis_to_rapier(axis_a);
        let rapier_axis_b = basis_to_rapier(axis_b);
        let space_handle = body_a.get_base().get_space_id();
        let space_id = body_a.get_base().get_space_id();
        let handle = physics_engine.joint_create_slider(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            rapier_anchor_a,
            rapier_anchor_b,
            rapier_axis_a,
            rapier_axis_b,
            0.0,
            0.0,
            false,
            false,
            true,
        );
        Self {
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle),
            linear_limit_lower: 0.0,
            linear_limit_upper: 0.0,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn set_param(
        &mut self,
        p_param: physics_server_3d::SliderJointParam,
        p_value: f32,
        physics_engine: &mut PhysicsEngine,
    ) {
        match p_param {
            physics_server_3d::SliderJointParam::LINEAR_LIMIT_UPPER => {
                self.linear_limit_upper = p_value
            }
            physics_server_3d::SliderJointParam::LINEAR_LIMIT_LOWER => {
                self.linear_limit_lower = p_value
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.joint_change_slider(
            self.base.get_space_id(),
            self.base.get_handle(),
            self.linear_limit_lower,
            self.linear_limit_upper,
        );
    }

    pub fn get_param(&self, p_param: physics_server_3d::SliderJointParam) -> f32 {
        match p_param {
            physics_server_3d::SliderJointParam::LINEAR_LIMIT_UPPER => self.linear_limit_upper,
            physics_server_3d::SliderJointParam::LINEAR_LIMIT_LOWER => self.linear_limit_lower,
            _ => 0.0,
        }
    }
}
impl IRapierJoint for RapierSliderJoint3D {
    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    fn get_type(&self) -> physics_server_3d::JointType {
        physics_server_3d::JointType::SLIDER
    }
}
