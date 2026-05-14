use godot::global::godot_error;
use rapier::prelude::*;

use crate::joints::rapier_joint_base::RapierJointType;
#[cfg(feature = "dim3")]
use crate::rapier_wrapper::joint::glamx::Quat;
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
        target_transform: Pose,
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
        anchor_1: Vector,
        anchor_2: Vector,
        angular_limit_lower: Real,
        angular_limit_upper: Real,
        angular_limit_enabled: bool,
        motor_target_velocity: Real,
        motor_enabled: bool,
        joint_type: RapierJointType,
        motor_target_position: Real,
        motor_stiffness: Real,
        motor_damping: Real,
        motor_position_enabled: bool,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let mut joint = RevoluteJointBuilder::new()
                .local_anchor1(anchor_1)
                .local_anchor2(anchor_2)
                .contacts_enabled(!disable_collision)
                .motor_max_force(Real::MAX)
                .motor_model(MotorModel::ForceBased);
            if angular_limit_enabled {
                joint = joint.limits([angular_limit_lower, angular_limit_upper]);
            }
            if motor_enabled {
                joint = joint.motor_velocity(motor_target_velocity, 0.0)
            } else if motor_position_enabled {
                joint = joint.motor_position(motor_target_position, motor_stiffness, motor_damping)
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
        anchor_1: Vector,
        anchor_2: Vector,
        axis_1: Rotation,
        axis_2: Rotation,
        angular_limit_lower: Real,
        angular_limit_upper: Real,
        angular_limit_enabled: bool,
        motor_target_velocity: Real,
        motor_enabled: bool,
        joint_type: RapierJointType,
        motor_target_position: Real,
        motor_stiffness: Real,
        motor_damping: Real,
        motor_position_enabled: bool,
        motor_max_force: Real,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // Rapier revolute hinge axis is X, but godot is Z
            // Transforming poses from z to x so that we can use the simplified Rapier
            // revolute joint functions in other parts of code
            // (rather than setting up a GenericJoint with AngZ free axis)
            let z_to_x =
                Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), std::f32::consts::FRAC_PI_2);
            let pose_1 = Pose::from_parts(anchor_1, axis_1 * z_to_x);
            let pose_2 = Pose::from_parts(anchor_2, axis_2 * z_to_x);
            // Use GenericJointBuilder to set both local axes
            let mut joint = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
                .local_frame1(pose_1)
                .local_frame2(pose_2)
                .contacts_enabled(!disable_collision);
            if angular_limit_enabled {
                joint = joint.limits(JointAxis::AngX, [angular_limit_lower, angular_limit_upper]);
            }
            if motor_enabled {
                joint = joint
                    .motor_velocity(JointAxis::AngX, motor_target_velocity, 0.0)
                    .motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
                    .motor_max_force(JointAxis::AngX, motor_max_force);
            } else if motor_position_enabled {
                joint = joint
                    .motor_position(
                        JointAxis::AngX,
                        motor_target_position,
                        motor_stiffness,
                        motor_damping,
                    )
                    .motor_max_force(JointAxis::AngX, Real::MAX)
                    .motor_model(JointAxis::AngX, MotorModel::AccelerationBased);
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
        anchor_1: Vector,
        anchor_2: Vector,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let joint = SphericalJointBuilder::new()
                .local_anchor1(anchor_1)
                .local_anchor2(anchor_2)
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
        anchor_1: Vector,
        anchor_2: Vector,
        axis_1: Rotation,
        axis_2: Rotation,
        linear_limit_upper: f32,
        linear_limit_lower: f32,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let pose_1 = Pose::from_parts(anchor_1, axis_1);
            let pose_2 = Pose::from_parts(anchor_2, axis_2);
            // Use GenericJointBuilder to set both local axes for prismatic joint
            let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_PRISMATIC_AXES)
                .local_frame1(pose_1)
                .local_frame2(pose_2)
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
        anchor_1: Vector,
        anchor_2: Vector,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
            && let Some(joint) = joint.as_spherical_mut()
        {
            joint
                .set_local_anchor1(anchor_1)
                .set_local_anchor2(anchor_2);
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
        motor_target_position: Real,
        motor_stiffness: Real,
        motor_damping: Real,
        motor_position_enabled: bool,
        #[cfg(feature = "dim3")] motor_max_force: Real,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
            && let Some(joint) = joint.as_revolute_mut()
        {
            joint.set_motor_model(MotorModel::AccelerationBased);
            joint.set_motor_max_force(Real::MAX);
            if angular_limit_enabled {
                joint.set_limits([angular_limit_lower, angular_limit_upper]);
            } else {
                joint.data.limit_axes.remove(JointAxesMask::ANG_X);
            }
            if motor_enabled {
                joint.set_motor(0.0, motor_target_velocity, 0.0, 0.0);
                #[cfg(feature = "dim3")]
                joint.set_motor_max_force(motor_max_force);
            } else if motor_position_enabled {
                joint.set_motor(motor_target_position, 0.0, motor_stiffness, motor_damping);
            } else {
                joint.data.motor_axes.remove(JointAxesMask::ANG_X);
            }
            if softness <= 0.0 {
                joint.data.softness.natural_frequency = 1.0e6;
                joint.data.softness.damping_ratio = 1.0;
            } else {
                // Convert softness to damping parameters
                let softness_clamped = softness.clamp(Real::EPSILON, 16.0);
                joint.data.softness.natural_frequency = 10_f32.powf(3.0 - softness_clamped * 0.2);
                joint.data.softness.damping_ratio = 10_f32.powf(-softness_clamped * 0.4375);
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
        axis: Vector,
        anchor_1: Vector,
        anchor_2: Vector,
        limits: Vector,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let joint = PinSlotJointBuilder::new(axis)
                .local_anchor1(anchor_1)
                .local_anchor2(anchor_2)
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
        anchor_1: Vector,
        anchor_2: Vector,
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
                .local_anchor1(anchor_1)
                .local_anchor2(anchor_2)
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
        anchor_1: Vector,
        anchor_2: Vector,
        axis_1: Rotation,
        axis_2: Rotation,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // NOTE: 6DOF Angle limits may not behave as expected or be
            // unstable when there is more than one angular DOF.
            //
            // Jolt seems to use a swing twist constraint for it's 6DOF which
            // improves the situation by eliminating path dependence problems.
            // If Rapier could add a pyramid like constraint to it's coupled angular
            // axes then we could match what Jolt is doing.
            // As of Rapier 0.32.0, I don't believe we can match jolt's limit behavior.
            //
            // For now, if a user needs high angular DOF's with limits, they
            // can construct a compound joint out of multiple low DOF joints
            // with dummy rigid bodies between them.
            //
            // They could also use hidden colliders as a constraint, instead of joint limits
            //
            let pose_1 = Pose::from_parts(anchor_1, axis_1);
            let pose_2 = Pose::from_parts(anchor_2, axis_2);
            let joint = GenericJointBuilder::new(JointAxesMask::FREE_FIXED_AXES)
                .local_frame1(pose_1)
                .local_frame2(pose_2)
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
        anchor_1: Vector,
        anchor_2: Vector,
        axis_1: Rotation,
        axis_2: Rotation,
        swing_span: Real,
        twist_span: Real,
        joint_type: RapierJointType,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // A common use for this joint seems to be a shoulder for rag dolls.
            // The arm held out to a middle position might represent the x axis (twist)
            // The y and z rotation limits offer a kind of cone around that axis
            // for the arm to move in (swing)
            //
            // Rapier's coupled_axes feature makes the whole joint work as intended
            //
            // Note that Godot Docs and the widget that Godot draws (as of 4.6.2)
            // do not match what Jolt and the default engine are doing.
            let pose_1 = Pose::from_parts(anchor_1, axis_1);
            let pose_2 = Pose::from_parts(anchor_2, axis_2);
            let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES)
                .local_frame1(pose_1)
                .local_frame2(pose_2)
                .coupled_axes(JointAxesMask::ANG_Y | JointAxesMask::ANG_Z)
                .limits(JointAxis::AngX, [-twist_span, twist_span])
                .limits(JointAxis::AngY, [-swing_span, swing_span])
                .limits(JointAxis::AngZ, [-swing_span, swing_span])
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
            joint.set_limits(JointAxis::AngX, [-twist_span, twist_span]);
            joint.set_limits(JointAxis::AngY, [-swing_span, swing_span]);
            joint.set_limits(JointAxis::AngZ, [-swing_span, swing_span]);
        }
    }

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
        enable_limit: bool,
    ) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
        {
            // motor velocity should override motor position settings to match
            // how Jolt behaves.
            if enable_limit {
                joint.set_limits(axis, [lower_limit, limit_upper]);
            } else {
                joint.limit_axes.remove(JointAxesMask::from(axis));
            }
            if enable_motor {
                joint.set_motor_model(axis, MotorModel::ForceBased);
                joint.set_motor_max_force(axis, motor_force_limit);
                joint.set_motor(axis, 0.0, motor_target_velocity, 0.0, 0.0);
            } else if enable_spring {
                joint.set_motor_model(axis, MotorModel::AccelerationBased);
                joint.set_motor(
                    axis,
                    spring_equilibrium_point,
                    0.0,
                    spring_stiffness,
                    spring_damping,
                );
                joint.set_motor_max_force(axis, Real::MAX);
            } else {
                joint.motor_axes.remove(JointAxesMask::from(axis));
            }
        }
    }
}
