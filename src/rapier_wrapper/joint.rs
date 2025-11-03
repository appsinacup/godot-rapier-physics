use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
impl PhysicsEngine {
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
        multibody: bool,
        kinematic: bool,
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
            }
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
            );
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
        angular_limit_lower: Real,
        angular_limit_upper: Real,
        angular_limit_enabled: bool,
        motor_target_velocity: Real,
        motor_enabled: bool,
        multibody: bool,
        kinematic: bool,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let axis = anchor_1 - anchor_2;
            let unit_axis = UnitVector::new_normalize(axis.normalize());
            let mut joint = RevoluteJointBuilder::new(unit_axis)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .contacts_enabled(!disable_collision);
            if angular_limit_enabled {
                joint = joint.limits([angular_limit_lower, angular_limit_upper]);
            }
            if motor_enabled {
                joint = joint.motor_velocity(motor_target_velocity, 0.0);
            }
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
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
        multibody: bool,
        kinematic: bool,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let joint = SphericalJointBuilder::new()
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
            );
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
        linear_limit_upper: f32,
        linear_limit_lower: f32,
        multibody: bool,
        kinematic: bool,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let axis = anchor_1 - anchor_2;
            let unit_axis = UnitVector::new_normalize(axis.normalize());
            let joint = PrismaticJointBuilder::new(unit_axis)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .limits([linear_limit_lower, linear_limit_upper])
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
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
        multibody: bool,
        kinematic: bool,
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
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
            );
        }
        JointHandle::default()
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "dim2")]
    pub fn joint_create_prismatic(
        &mut self,
        world_handle: WorldHandle,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        axis: Vector<Real>,
        anchor_1: Vector<Real>,
        anchor_2: Vector<Real>,
        limits: Vector<Real>,
        multibody: bool,
        kinematic: bool,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let joint = PrismaticJointBuilder::new(UnitVector::new_unchecked(axis))
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .limits([limits.x, limits.y])
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
            );
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
        multibody: bool,
        kinematic: bool,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let joint = SpringJointBuilder::new(rest_length, stiffness, damping)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
            );
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
            joint.set_motor_position(JointAxis::LinX, rest_length, stiffness, damping);
            joint.set_motor_model(JointAxis::LinX, MotorModel::AccelerationBased);
        }
    }

    pub fn destroy_joint(&mut self, world_handle: WorldHandle, joint_handle: JointHandle) {
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.remove_joint(joint_handle);
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
        multibody: bool,
        kinematic: bool,
        disable_collision: bool,
    ) -> JointHandle {
        self.body_wake_up(world_handle, body_handle_1, false);
        self.body_wake_up(world_handle, body_handle_2, false);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            // Create a generic joint that allows all 6 degrees of freedom
            // Extract the X axis from the rotation as UnitVector
            let axis1_vec = axis_1 * Vector::x_axis();
            let axis2_vec = axis_2 * Vector::x_axis();
            let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES)
                .local_anchor1(Point { coords: anchor_1 })
                .local_anchor2(Point { coords: anchor_2 })
                .local_axis1(axis1_vec)
                .local_axis2(axis2_vec)
                .contacts_enabled(!disable_collision);
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
            );
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
        multibody: bool,
        kinematic: bool,
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
            return physics_world.insert_joint(
                body_handle_1,
                body_handle_2,
                multibody,
                kinematic,
                joint,
            );
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

    #[cfg(feature = "dim3")]
    pub fn joint_change_generic_6dof_axis_param(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        axis: JointAxis,
        param: godot::classes::physics_server_3d::G6dofJointAxisParam,
        value: Real,
    ) {
        use godot::classes::physics_server_3d::G6dofJointAxisParam;
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
        {
            match param {
                G6dofJointAxisParam::LINEAR_LOWER_LIMIT => {
                    if let Some(limits) = joint.limits(axis) {
                        joint.set_limits(axis, [value, limits.max]);
                    } else {
                        joint.set_limits(axis, [value, Real::MAX]);
                    }
                }
                G6dofJointAxisParam::LINEAR_UPPER_LIMIT => {
                    if let Some(limits) = joint.limits(axis) {
                        joint.set_limits(axis, [limits.min, value]);
                    } else {
                        joint.set_limits(axis, [-Real::MAX, value]);
                    }
                }
                G6dofJointAxisParam::ANGULAR_LOWER_LIMIT => {
                    if let Some(limits) = joint.limits(axis) {
                        joint.set_limits(axis, [value, limits.max]);
                    } else {
                        joint.set_limits(axis, [value, Real::MAX]);
                    }
                }
                G6dofJointAxisParam::ANGULAR_UPPER_LIMIT => {
                    if let Some(limits) = joint.limits(axis) {
                        joint.set_limits(axis, [limits.min, value]);
                    } else {
                        joint.set_limits(axis, [-Real::MAX, value]);
                    }
                }
                G6dofJointAxisParam::LINEAR_MOTOR_TARGET_VELOCITY => {
                    joint.set_motor_velocity(axis, value, 0.0);
                }
                G6dofJointAxisParam::LINEAR_MOTOR_FORCE_LIMIT => {
                    joint.set_motor_max_force(axis, value);
                }
                G6dofJointAxisParam::ANGULAR_MOTOR_TARGET_VELOCITY => {
                    joint.set_motor_velocity(axis, value, 0.0);
                }
                G6dofJointAxisParam::ANGULAR_MOTOR_FORCE_LIMIT => {
                    joint.set_motor_max_force(axis, value);
                }
                G6dofJointAxisParam::LINEAR_SPRING_STIFFNESS => {
                    // Spring stiffness needs to be set with motor_position
                    let motor = joint.motors[axis as usize];
                    joint.set_motor_position(axis, motor.target_pos, value, motor.damping);
                }
                G6dofJointAxisParam::LINEAR_SPRING_DAMPING => {
                    let motor = joint.motors[axis as usize];
                    joint.set_motor_position(axis, motor.target_pos, motor.stiffness, value);
                }
                G6dofJointAxisParam::LINEAR_SPRING_EQUILIBRIUM_POINT => {
                    let motor = joint.motors[axis as usize];
                    joint.set_motor_position(axis, value, motor.stiffness, motor.damping);
                }
                G6dofJointAxisParam::ANGULAR_SPRING_STIFFNESS => {
                    let motor = joint.motors[axis as usize];
                    joint.set_motor_position(axis, motor.target_pos, value, motor.damping);
                }
                G6dofJointAxisParam::ANGULAR_SPRING_DAMPING => {
                    let motor = joint.motors[axis as usize];
                    joint.set_motor_position(axis, motor.target_pos, motor.stiffness, value);
                }
                G6dofJointAxisParam::ANGULAR_SPRING_EQUILIBRIUM_POINT => {
                    let motor = joint.motors[axis as usize];
                    joint.set_motor_position(axis, value, motor.stiffness, motor.damping);
                }
                _ => {}
            }
        }
    }

    #[cfg(feature = "dim3")]
    pub fn joint_change_generic_6dof_axis_flag(
        &mut self,
        world_handle: WorldHandle,
        joint_handle: JointHandle,
        axis: JointAxis,
        flag: godot::classes::physics_server_3d::G6dofJointAxisFlag,
        enable: bool,
    ) {
        use godot::classes::physics_server_3d::G6dofJointAxisFlag;
        self.joint_wake_up_connected_rigidbodies(world_handle, joint_handle);
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(joint) = physics_world.get_mut_joint(joint_handle)
        {
            match flag {
                G6dofJointAxisFlag::ENABLE_LINEAR_LIMIT
                | G6dofJointAxisFlag::ENABLE_ANGULAR_LIMIT => {
                    if enable {
                        // Enable limits by setting default limits if not already set
                        if joint.limits(axis).is_none() {
                            joint.set_limits(axis, [-1.0, 1.0]);
                        }
                    } else {
                        // Disable limits by setting them to max range
                        joint.set_limits(axis, [-Real::MAX, Real::MAX]);
                    }
                }
                G6dofJointAxisFlag::ENABLE_MOTOR | G6dofJointAxisFlag::ENABLE_LINEAR_MOTOR => {
                    if enable {
                        joint.set_motor_velocity(axis, 0.0, 0.0);
                        joint.set_motor_max_force(axis, 0.0);
                    } else {
                        joint.set_motor_max_force(axis, 0.0);
                    }
                }
                G6dofJointAxisFlag::ENABLE_LINEAR_SPRING
                | G6dofJointAxisFlag::ENABLE_ANGULAR_SPRING => {
                    if enable {
                        joint.set_motor_position(axis, 0.0, 0.0, 0.0);
                        joint.set_motor_model(axis, MotorModel::AccelerationBased);
                    } else {
                        joint.set_motor_position(axis, 0.0, 0.0, 0.0);
                    }
                }
                _ => {}
            }
        }
    }
}
