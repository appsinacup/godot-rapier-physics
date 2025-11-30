use godot::global::godot_error;
use rapier::prelude::*;

use crate::joints::rapier_joint_base::RapierJointType;
use crate::rapier_wrapper::prelude::*;
impl PhysicsEngine {
    #[cfg(feature = "dim2")]
    fn godot_spring_to_rapier_accel(stiffness: Real, damping: Real) -> (Real, Real) {
        // Godot stiffness is in N/m, convert to frequency: omega = sqrt(k/m)
        // For AccelerationBased, assume unit mass (m=1)
        let omega = stiffness.sqrt();
        // Calculate damping ratio from Godot damping: zeta = c / (2 * sqrt(k*m))
        // For unit mass: zeta = c / (2 * sqrt(k))
        let damping_ratio = if stiffness > 0.0 {
            damping / (2.0 * stiffness.sqrt())
        } else {
            0.0
        };
        // Convert back to AccelerationBased stiffness/damping
        let rapier_stiffness = omega * omega;
        let rapier_damping = 2.0 * damping_ratio * omega;
        (rapier_stiffness, rapier_damping)
    }

    pub fn get_multibody_rigidbodies(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
    ) -> Vec<RigidBodyHandle> {
        let mut result = Vec::new();
        let Some(physics_world) = self.get_mut_world(world_handle) else {
            godot_error!(
                "Failed to wake up multibody: world handle {:?} is not valid",
                world_handle
            );
            return result;
        };
        let Some((multibody, _)) = physics_world
            .physics_objects
            .multibody_joint_set
            .get_mut(MultibodyJointHandle(joint_handle.index))
        else {
            godot_error!(
                "Failed to solve IK: joint handle {:?} is not a valid multibody joint",
                joint_handle
            );
            return result;
        };
        for link in multibody.links() {
            result.push(link.rigid_body_handle());
        }
        result
    }

    pub fn multibody_solve_ik(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        target_transform: Isometry<Real>,
        options: InverseKinematicsOption,
    ) {
        let Some(physics_world) = self.get_mut_world(world_handle) else {
            godot_error!(
                "Failed to solve IK: world handle {:?} is not valid",
                world_handle
            );
            return;
        };
        let Some((multibody, link_id)) = physics_world
            .physics_objects
            .multibody_joint_set
            .get_mut(MultibodyJointHandle(joint_handle.index))
        else {
            godot_error!(
                "Failed to solve IK: joint handle {:?} is not a valid multibody joint",
                joint_handle
            );
            return;
        };
        let ndofs = multibody.ndofs();
        if ndofs == 0 {
            godot_error!("Cannot solve IK: multibody has 0 degrees of freedom");
            return;
        }
        let mut displacements = nalgebra::DVector::zeros(ndofs);
        multibody.inverse_kinematics(
            &physics_world.physics_objects.rigid_body_set,
            link_id,
            &options,
            &target_transform,
            |_| true,
            &mut displacements,
        );
        // Validate displacements are finite before applying
        if !displacements.iter().all(|&d| d.is_finite()) {
            godot_error!("IK solver produced non-finite displacements, skipping application");
            return;
        }
        multibody.apply_displacements(displacements.as_slice());
    }

    fn joint_wake_up_connected_rigidbodies(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
    ) {
        let mut body1 = None;
        let mut body2 = None;
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(joint) = physics_world.get_impulse_joint(joint_handle)
        {
            body1 = Some(joint.body1);
            body2 = Some(joint.body2);
        }
        if let Some(body1) = body1
            && let Some(body2) = body2
        {
            self.body_wake_up(world_handle, body1, false);
            self.body_wake_up(world_handle, body2, false);
        }
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn joint_create_revolute(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        angular_limit_lower: Real,
        angular_limit_upper: Real,
        angular_limit_enabled: bool,
        motor_target_velocity: Real,
        motor_enabled: bool,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let mut joint = RevoluteJointBuilder::new()
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .contacts_enabled(!disable_collision);
            if angular_limit_enabled {
                joint = joint.limits([angular_limit_lower, angular_limit_upper]);
            }
            if motor_enabled {
                joint = joint.motor_velocity(motor_target_velocity, 0.0);
                joint = joint.motor_model(MotorModel::AccelerationBased);
            }
            return physics_world.insert_joint(body_handle_1, body_handle_2, joint_type, joint);
        }
        JointHandle::default()
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn joint_create_revolute(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        axis_1: Rotation<Real>,
        axis_2: Rotation<Real>,
        angular_limit_lower: Real,
        angular_limit_upper: Real,
        angular_limit_enabled: bool,
        motor_target_velocity: Real,
        motor_enabled: bool,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // Extract the hinge axis (X-axis) from the rotation matrices
            let axis1_vec = axis_1 * Vector::x_axis();
            let axis2_vec = axis_2 * Vector::x_axis();
            // Use GenericJointBuilder to set both local axes
            let mut joint = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .local_axis1(axis1_vec)
                .local_axis2(axis2_vec)
                .contacts_enabled(!disable_collision);
            if angular_limit_enabled {
                joint = joint.limits(JointAxis::AngX, [angular_limit_lower, angular_limit_upper]);
            }
            if motor_enabled {
                joint = joint.motor_velocity(JointAxis::AngX, motor_target_velocity, 0.0);
            }
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                joint_type,
                joint.build(),
            );
        }
        JointHandle::default()
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn joint_create_spherical(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let joint = SphericalJointBuilder::new()
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(body_handle_1, body_handle_2, joint_type, joint);
        }
        JointHandle::default()
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn joint_create_slider(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        axis_1: Rotation<Real>,
        axis_2: Rotation<Real>,
        linear_limit_upper: f32,
        linear_limit_lower: f32,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // Extract the X axis from the rotation matrices
            let axis1_vec = axis_1 * Vector::x_axis();
            let axis2_vec = axis_2 * Vector::x_axis();
            // Use GenericJointBuilder to set both local axes for prismatic joint
            let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_PRISMATIC_AXES)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .local_axis1(axis1_vec)
                .local_axis2(axis2_vec)
                .limits(JointAxis::LinX, [linear_limit_lower, linear_limit_upper])
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                joint_type,
                joint.build(),
            );
        }
        JointHandle::default()
    }

    #[cfg(feature = "dim3")]
    pub fn joint_change_slider(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        linear_limit_lower: Real,
        linear_limit_upper: Real,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
            && let Some(joint) = joint.as_prismatic_mut()
        {
            joint.set_limits([linear_limit_lower, linear_limit_upper]);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn join_change_sperical_anchors(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
            && let Some(joint) = joint.as_spherical_mut()
        {
            joint
                .set_local_anchor1(Point { coords: anchor_1 })
                .set_local_anchor2(Point { coords: anchor_2 });
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn joint_change_revolute_params(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        angular_limit_lower: Real,
        angular_limit_upper: Real,
        angular_limit_enabled: bool,
        motor_target_velocity: Real,
        motor_enabled: bool,
        softness: Real,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
            && let Some(joint) = joint.as_revolute_mut()
        {
            if motor_enabled {
                joint
                    .set_motor_velocity(motor_target_velocity, 0.0)
                    .set_motor_max_force(Real::MAX);
            } else {
                joint.set_motor_velocity(0.0, 0.0).set_motor_max_force(0.0);
            }
            if angular_limit_enabled {
                joint.set_limits([angular_limit_lower, angular_limit_upper]);
            }
            if softness <= 0.0 {
                joint.data.natural_frequency = 1.0e6;
                joint.data.damping_ratio = 1.0;
            } else {
                // Convert softness to damping parameters
                let softness_clamped = softness.clamp(Real::EPSILON, 16.0);
                joint.data.natural_frequency = 10_f32.powf(3.0 - softness_clamped * 0.2);
                joint.data.damping_ratio = 10_f32.powf(-softness_clamped * 0.4375);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "dim2")]
    pub fn joint_create_pin_slot(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        axis: Vector<Real>,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        limits: Vector<Real>,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let joint = PinSlotJointBuilder::new(UnitVector::new_unchecked(axis))
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .limits([limits.x, limits.y])
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(body_handle_1, body_handle_2, joint_type, joint);
        }
        JointHandle::default()
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "dim2")]
    pub fn joint_create_spring(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        stiffness: Real,
        damping: Real,
        rest_length: Real,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let (rapier_stiffness, rapier_damping) =
                Self::godot_spring_to_rapier_accel(stiffness, damping);
            let joint = SpringJointBuilder::new(rest_length, rapier_stiffness, rapier_damping)
                .spring_model(MotorModel::AccelerationBased)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(body_handle_1, body_handle_2, joint_type, joint);
        }
        JointHandle::default()
    }

    #[cfg(feature = "dim2")]
    pub fn joint_change_spring_params(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        stiffness: Real,
        damping: Real,
        rest_length: Real,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
        {
            let (rapier_stiffness, rapier_damping) =
                Self::godot_spring_to_rapier_accel(stiffness, damping);
            joint.set_motor_position(
                JointAxis::LinX,
                rest_length,
                rapier_stiffness,
                rapier_damping,
            );
            joint.set_motor_model(JointAxis::LinX, MotorModel::AccelerationBased);
        }
    }

    pub fn destroy_joint(&mut self, world_handle: WorldHandle, joint_handle: JointHandle) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.remove_joint(joint_handle);
        }
    }

    pub fn recreate_joint(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        new_joint_type: RapierJointType,
    ) -> JointHandle {
        // Get the joint data and connected bodies
        let (joint_data, body1, body2) = if let Some(physics_world) = self.get_world(world_handle) {
            if let Some(joint) = physics_world.get_joint(joint_handle) {
                if let Some((body1, body2)) = physics_world.get_joint_bodies(joint_handle) {
                    (*joint, body1, body2)
                } else {
                    godot_error!("Invalid joint bodies");
                    return JointHandle::default();
                }
            } else {
                godot_error!("Invalid joint");
                return JointHandle::default();
            }
        } else {
            return JointHandle::default();
        };
        if body1 == RigidBodyHandle::default() || body2 == RigidBodyHandle::default() {
            godot_error!("Invalid joint bodies");
            return JointHandle::default();
        }
        // Wake up connected bodies
        self.body_wake_up(world_handle, body1, false);
        self.body_wake_up(world_handle, body2, false);
        // Remove the old joint
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.remove_joint(joint_handle);
        }
        // Insert the joint into the new set
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.insert_joint(body1, body2, new_joint_type, joint_data)
        } else {
            godot_error!("Invalid joint data");
            JointHandle::default()
        }
    }

    pub fn joint_change_disable_collision(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        disable_collision: bool,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
        {
            joint.set_contacts_enabled(!disable_collision);
        }
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn joint_create_generic_6dof(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        axis_1: Rotation<Real>,
        axis_2: Rotation<Real>,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // Create a generic joint that allows all 6 degrees of freedom
            // Extract the X axis from the rotation as UnitVector
            let axis1_vec = axis_1 * Vector::x_axis();
            let axis2_vec = axis_2 * Vector::x_axis();
            let joint = GenericJointBuilder::new(JointAxesMask::FREE_FIXED_AXES)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .local_axis1(axis1_vec)
                .local_axis2(axis2_vec)
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(body_handle_1, body_handle_2, joint_type, joint);
        }
        JointHandle::default()
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn joint_create_cone_twist(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        axis_1: Rotation<Real>,
        axis_2: Rotation<Real>,
        swing_span: Real,
        twist_span: Real,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // Approximate cone: limit swing on X and Y to create a "box" that fits inside the cone
            // Use swing_span / sqrt(2) to get the per-axis limit
            let swing_limit = swing_span / 2.0f32.sqrt();
            let twist_limit = twist_span / 2.0;
            // Extract the X axis from the rotation as UnitVector
            let axis1_vec = axis_1 * Vector::x_axis();
            let axis2_vec = axis_2 * Vector::x_axis();
            // Create a generic joint with locked translations and limited rotations
            let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .local_axis1(axis1_vec)
                .local_axis2(axis2_vec)
                .limits(JointAxis::AngX, [-swing_limit, swing_limit])
                .limits(JointAxis::AngY, [-swing_limit, swing_limit])
                .limits(JointAxis::AngZ, [-twist_limit, twist_limit])
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(body_handle_1, body_handle_2, joint_type, joint);
        }
        JointHandle::default()
    }

    #[cfg(feature = "dim3")]
    pub fn joint_change_cone_twist_params(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        swing_span: Real,
        twist_span: Real,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
        {
            let swing_limit = swing_span / 2.0f32.sqrt();
            let twist_limit = twist_span / 2.0;
            joint.set_limits(JointAxis::AngX, [-swing_limit, swing_limit]);
            joint.set_limits(JointAxis::AngY, [-swing_limit, swing_limit]);
            joint.set_limits(JointAxis::AngZ, [-twist_limit, twist_limit]);
        }
    }

    //TODO: Remove motor_enabled and spring_enabled once we have guidance on how to implement this properly
    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn joint_change_generic_6dof_axis_param(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        axis: JointAxis,
        lower_limit: Real,
        limit_upper: Real,
        enable_motor: bool,
        motor_target_velocity: Real,
        motor_force_limit: Real,
        enable_spring: bool,
        spring_damping: Real,
        spring_stiffness: Real,
        spring_equilibrium_point: Real,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
        {
            if enable_motor && enable_spring {
                godot::global::godot_warn!(
                    "Both spring and motor model are enabled on joint, this is currently not implemented, motor will be given precidence!"
                );
            }
            joint.set_limits(axis, [lower_limit, limit_upper]);
            if enable_motor {
                joint.set_motor_model(axis, MotorModel::ForceBased);
                joint.set_motor_max_force(axis, motor_force_limit);
                //TODO: where do we want to get the damping factor from: Higher in this case means the target velocity is approched more aggresively
                joint.set_motor_velocity(axis, motor_target_velocity, 10.0);
            } else if enable_spring {
                joint.set_motor_model(axis, MotorModel::AccelerationBased);
                joint.set_motor_position(
                    axis,
                    spring_equilibrium_point,
                    spring_stiffness,
                    spring_damping,
                );
            } else {
                joint.set_motor_max_force(axis, 0.0);
                joint.set_motor(axis, 0.0, 0.0, 0.0, 0.0);
            }
        }
    }
}
