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
        _world_handle: WorldHandle,
        _body_handle_1: RigidBodyHandle,
        _body_handle_2: RigidBodyHandle,
        _anchor_1: Vector<Real>,
        _anchor_2: Vector<Real>,
        _multibody: bool,
        _kinematic: bool,
        _disable_collision: bool,
    ) -> JointHandle {
        // TODO
        JointHandle::default()
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
}
