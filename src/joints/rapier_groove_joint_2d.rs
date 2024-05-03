use crate::{
    joints::rapier_joint_2d::IRapierJoint2D,
    rapier2d::{joint::joint_create_prismatic, vector::Vector},
    servers::rapier_physics_singleton_2d::physics_singleton,
};
use godot::{
    builtin::{Rid, Vector2},
    engine::physics_server_2d,
};

use super::rapier_joint_2d::RapierJointBase2D;
pub struct RapierGrooveJoint2D {
    base: RapierJointBase2D,
}

impl RapierGrooveJoint2D {
    pub fn new(
        rid: Rid,
        p_a_groove1: Vector2,
        p_a_groove2: Vector2,
        p_b_anchor: Vector2,
        body_a: Rid,
        body_b: Rid,
    ) -> Self {
        let lock = physics_singleton().lock().unwrap();
        let body_a = lock.collision_objects.get(&body_a);
        assert!(body_a.is_some());
        let body_a = body_a.unwrap();
        let base_a = body_a.get_base();
        let point_a_1 = base_a.get_inv_transform().basis_xform(p_a_groove1);
        let point_a_2 = base_a.get_inv_transform().basis_xform(p_a_groove2);
        let body_a_handle = body_a.get_base().get_body_handle();
        let axis = (point_a_2 - point_a_1).normalized();
        let length = (point_a_2 - point_a_1).length();
        let rapier_axis = Vector::new(axis.x, axis.y);
        let rapier_limits = Vector::new(0.0, length);
        let rapier_anchor_a = Vector::new(point_a_1.x, point_a_1.y);

        let space_a = lock.spaces.get(&body_a.get_base().get_space());
        assert!(space_a.is_some());
        let space_handle_a = space_a.unwrap().get_handle();

        let body_b = lock.collision_objects.get(&body_b);
        assert!(body_b.is_some());
        let body_b = body_b.unwrap();
        let body_b_handle = body_b.get_base().get_body_handle();
        let base_b = body_b.get_base();

        let anchor_b = base_b.get_inv_transform().basis_xform(p_b_anchor);
        let rapier_anchor_b = Vector::new(anchor_b.x, anchor_b.y);
        let handle = joint_create_prismatic(
            space_handle_a,
            body_a_handle,
            body_b_handle,
            &rapier_axis,
            &rapier_anchor_a,
            &rapier_anchor_b,
            &rapier_limits,
            true,
        );

        Self {
            base: RapierJointBase2D::new(space_handle_a, handle, rid),
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
