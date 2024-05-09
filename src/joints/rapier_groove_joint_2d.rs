use crate::{
    joints::rapier_joint_2d::IRapierJoint2D,
    rapier2d::{joint::joint_create_prismatic, vector::Vector},
    servers::rapier_physics_singleton_2d::{bodies_singleton, spaces_singleton},
};
use godot::{
    builtin::{Rid, Vector2},
    engine::physics_server_2d,
};

use super::{rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D, rapier_joint_2d::RapierJointBase2D, rapier_pin_joint_2d::RapierPinJoint2D};
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
        let body_a_handle;
        let body_b_handle;
        let rapier_axis;
        let rapier_anchor_a;
        let rapier_anchor_b;
        let rapier_limits;
        let space_rid;
        let space_handle;
        {
            let lock = bodies_singleton().lock().unwrap();
            let body_a = lock.collision_objects.get(&body_a);
            assert!(body_a.is_some());
            let body_a = body_a.unwrap();
            let base_a = body_a.get_base();
            let point_a_1 = base_a.get_inv_transform().basis_xform(p_a_groove1);
            let point_a_2 = base_a.get_inv_transform().basis_xform(p_a_groove2);
            body_a_handle = body_a.get_base().get_body_handle();
            let axis = (point_a_2 - point_a_1).normalized();
            let length = (point_a_2 - point_a_1).length();
            rapier_axis = Vector::new(axis.x, axis.y);
            rapier_limits = Vector::new(0.0, length);
            rapier_anchor_a = Vector::new(point_a_1.x, point_a_1.y);

            space_rid = body_a.get_base().get_space();

            let body_b = lock.collision_objects.get(&body_b);
            assert!(body_b.is_some());
            let body_b = body_b.unwrap();
            body_b_handle = body_b.get_base().get_body_handle();
            let base_b = body_b.get_base();

            let anchor_b = base_b.get_inv_transform().basis_xform(p_b_anchor);
            rapier_anchor_b = Vector::new(anchor_b.x, anchor_b.y);
        }
        {
            let lock = spaces_singleton().lock().unwrap();
            let space_a = lock.spaces.get(&space_rid);
            assert!(space_a.is_some());
            space_handle = space_a.unwrap().get_handle();
        }

        let handle = joint_create_prismatic(
            space_handle,
            body_a_handle,
            body_b_handle,
            &rapier_axis,
            &rapier_anchor_a,
            &rapier_anchor_b,
            &rapier_limits,
            true,
        );

        Self {
            base: RapierJointBase2D::new(space_handle, handle, rid),
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

    fn get_mut_base(&mut self) -> &mut RapierJointBase2D {
        &mut self.base
    }

    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D> {
        None
    }
    fn get_pin(&self) -> Option<&RapierPinJoint2D> {
        None
    }
    fn get_groove(&self) -> Option<&RapierGrooveJoint2D> {
        Some(self)
    }

    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }
    fn get_mut_pin(&mut self) -> Option<&mut RapierPinJoint2D> {
        None
    }
    fn get_mut_groove(&mut self) -> Option<&mut RapierGrooveJoint2D> {
        Some(self)
    }
}
