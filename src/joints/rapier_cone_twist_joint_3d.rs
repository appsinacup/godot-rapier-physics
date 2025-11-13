use godot::classes::*;
use godot::prelude::*;

use super::rapier_joint_base::RapierJointBase;
use super::rapier_joint_base::RapierJointType;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::*;
pub struct RapierConeTwistJoint3D {
    base: RapierJointBase,
    swing_span: f32,
    twist_span: f32,
    bias: f32,
    softness: f32,
    relaxation: f32,
}
impl RapierConeTwistJoint3D {
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
        joint_type: RapierJointType,
    ) -> Self {
        let invalid_joint = Self {
            base: RapierJointBase::default(),
            swing_span: std::f32::consts::PI / 4.0, // Default 45 degrees
            twist_span: std::f32::consts::PI,       // Default 180 degrees
            bias: 0.3,
            softness: 1.0,
            relaxation: 1.0,
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
        let swing_span = std::f32::consts::PI / 4.0;
        let twist_span = std::f32::consts::PI;
        let handle = physics_engine.joint_create_cone_twist(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            rapier_anchor_a,
            rapier_anchor_b,
            rapier_axis_a,
            rapier_axis_b,
            swing_span,
            twist_span,
            matches!(joint_type, RapierJointType::MultiBody),
            false,
            true,
        );
        Self {
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle, joint_type),
            swing_span,
            twist_span,
            bias: 0.3,
            softness: 1.0,
            relaxation: 1.0,
        }
    }

    pub fn set_param(
        &mut self,
        param: physics_server_3d::ConeTwistJointParam,
        value: f32,
        physics_engine: &mut PhysicsEngine,
    ) {
        match param {
            physics_server_3d::ConeTwistJointParam::SWING_SPAN => {
                self.swing_span = value;
            }
            physics_server_3d::ConeTwistJointParam::TWIST_SPAN => {
                self.twist_span = value;
            }
            physics_server_3d::ConeTwistJointParam::BIAS => {
                self.bias = value;
            }
            physics_server_3d::ConeTwistJointParam::SOFTNESS => {
                self.softness = value;
            }
            physics_server_3d::ConeTwistJointParam::RELAXATION => {
                self.relaxation = value;
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.joint_change_cone_twist_params(
            self.base.get_space_id(),
            self.base.get_handle(),
            self.swing_span,
            self.twist_span,
        );
    }

    pub fn get_param(&self, param: physics_server_3d::ConeTwistJointParam) -> f32 {
        match param {
            physics_server_3d::ConeTwistJointParam::SWING_SPAN => self.swing_span,
            physics_server_3d::ConeTwistJointParam::TWIST_SPAN => self.twist_span,
            physics_server_3d::ConeTwistJointParam::BIAS => self.bias,
            physics_server_3d::ConeTwistJointParam::SOFTNESS => self.softness,
            physics_server_3d::ConeTwistJointParam::RELAXATION => self.relaxation,
            _ => 0.0,
        }
    }
}
impl IRapierJoint for RapierConeTwistJoint3D {
    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    fn get_type(&self) -> physics_server_3d::JointType {
        physics_server_3d::JointType::CONE_TWIST
    }
}
