use nalgebra::Point;
use rapier::prelude::*;

use super::ANG_ZERO;
use crate::rapier_wrapper::prelude::*;
pub enum BodyType {
    Dynamic,
    Kinematic,
    Static,
}
fn set_rigid_body_properties_internal(
    rigid_body: &mut RigidBody,
    pos: Translation<Real>,
    rot: Rotation<Real>,
    teleport: bool,
    wake_up: bool,
) {
    if rigid_body.is_dynamic() || rigid_body.is_fixed() || teleport {
        rigid_body.set_position(Isometry::from_parts(pos, rot), wake_up);
    } else {
        rigid_body.set_next_kinematic_position(Isometry::from_parts(pos, rot));
    }
}
impl PhysicsEngine {
    fn body_wake_up_connected_rigidbodies(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            for (rb1, rb2, ..) in physics_world
                .physics_objects
                .impulse_joint_set
                .attached_joints(body_handle)
            {
                if let Some(rb1) = physics_world.physics_objects.rigid_body_set.get_mut(rb1) {
                    rb1.wake_up(true);
                }
                if let Some(rb2) = physics_world.physics_objects.rigid_body_set.get_mut(rb2) {
                    rb2.wake_up(true);
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn body_create(
        &mut self,
        world_handle: WorldHandle,
        pos: Vector<Real>,
        rot: Rotation<Real>,
        user_data: &UserData,
        body_type: BodyType,
        activation_angular_threshold: Real,
        activation_linear_threshold: Real,
        activation_time_until_sleep: Real,
    ) -> RigidBodyHandle {
        let Some(physics_world) = self.get_mut_world(world_handle) else {
            return RigidBodyHandle::invalid();
        };
        let mut rigid_body: RigidBody;
        match body_type {
            BodyType::Dynamic => {
                rigid_body = RigidBodyBuilder::dynamic().build();
            }
            BodyType::Kinematic => {
                rigid_body = RigidBodyBuilder::kinematic_position_based().build();
            }
            BodyType::Static => {
                rigid_body = RigidBodyBuilder::fixed().build();
            }
        }
        let activation = rigid_body.activation_mut();
        activation.angular_threshold = activation_angular_threshold;
        activation.normalized_linear_threshold = activation_linear_threshold;
        activation.time_until_sleep = activation_time_until_sleep;
        set_rigid_body_properties_internal(
            &mut rigid_body,
            Translation::from(pos),
            rot,
            true,
            true,
        );
        rigid_body.user_data = user_data.get_data();
        physics_world
            .physics_objects
            .rigid_body_set
            .insert(rigid_body)
    }

    pub fn body_change_mode(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        body_type: BodyType,
        wakeup: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            match body_type {
                BodyType::Dynamic => {
                    body.set_body_type(RigidBodyType::Dynamic, wakeup);
                }
                BodyType::Kinematic => {
                    body.set_body_type(RigidBodyType::KinematicPositionBased, wakeup);
                }
                BodyType::Static => {
                    body.set_body_type(RigidBodyType::Fixed, wakeup);
                }
            }
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_destroy(&mut self, world_handle: WorldHandle, body_handle: RigidBodyHandle) {
        let Some(physics_world) = self.get_mut_world(world_handle) else {
            return;
        };
        physics_world.remove_rigid_body(body_handle);
    }

    pub fn body_get_position(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) -> Vector<Real> {
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get(body_handle)
        {
            let body_vector = body.translation();
            return *body_vector;
        }
        Vector::default()
    }

    pub fn body_get_angle(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) -> Rotation<Real> {
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get(body_handle)
        {
            return *body.rotation();
        }
        Rotation::default()
    }

    pub fn body_set_transform(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        pixel_pos: Vector<Real>,
        rot: Rotation<Real>,
        teleport: bool,
        wake_up: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            set_rigid_body_properties_internal(
                body,
                Translation::from(pixel_pos),
                rot,
                teleport,
                wake_up,
            );
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_get_linear_velocity(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) -> Vector<Real> {
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get(body_handle)
        {
            let body_vel = body.linvel();
            return *body_vel;
        }
        Vector::default()
    }

    pub fn body_set_linear_velocity(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        vel: Vector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.set_linvel(vel, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_set_axis_lock(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        axis_lock: LockedAxes,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.set_locked_axes(axis_lock, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_update_material(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        mat: &Material,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            for collider in body.colliders() {
                if let Some(col) = physics_world
                    .physics_objects
                    .collider_set
                    .get_mut(*collider)
                {
                    col.set_friction(mat.friction);
                    col.set_restitution(mat.restitution);
                    col.set_contact_skin(mat.contact_skin);
                    col.set_collision_groups(InteractionGroups {
                        memberships: Group::from(mat.collision_layer),
                        filter: Group::from(mat.collision_mask),
                    });
                }
            }
            body.wake_up(false);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    #[cfg(feature = "dim2")]
    pub fn body_get_angular_velocity(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) -> AngVector<Real> {
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get(body_handle)
        {
            return body.angvel();
        }
        ANG_ZERO
    }

    #[cfg(feature = "dim3")]
    pub fn body_get_angular_velocity(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) -> AngVector<Real> {
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get(body_handle)
        {
            return *body.angvel();
        }
        ANG_ZERO
    }

    pub fn body_set_angular_velocity(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        vel: AngVector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.set_angvel(vel, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_set_linear_damping(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        linear_damping: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.set_linear_damping(linear_damping);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_set_angular_damping(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        angular_damping: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.set_angular_damping(angular_damping);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_set_gravity_scale(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        gravity_scale: Real,
        wake_up: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.set_gravity_scale(gravity_scale, wake_up);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_set_can_sleep(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        can_sleep: bool,
        activation_angular_threshold: Real,
        activation_linear_threshold: Real,
        activation_time_until_sleep: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            if !can_sleep {
                let activation = body.activation_mut();
                activation.angular_threshold = -1.0;
                activation.normalized_linear_threshold = -1.0;
            } else {
                let activation = body.activation_mut();
                activation.angular_threshold = activation_angular_threshold;
                activation.normalized_linear_threshold = activation_linear_threshold;
                activation.time_until_sleep = activation_time_until_sleep;
            }
            if !can_sleep && body.is_sleeping() {
                body.wake_up(true);
            }
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_set_ccd_enabled(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        enable: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.enable_ccd(enable);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn body_set_mass_properties(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        mass: Real,
        inertia: AngVector<Real>,
        local_com: Vector<Real>,
        wake_up: bool,
        force_update: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            #[cfg(feature = "dim2")]
            if inertia == ANG_ZERO {
                body.lock_rotations(true, wake_up);
            } else {
                body.lock_rotations(false, wake_up);
            }
            for collider in body.colliders() {
                if let Some(collider) = physics_world
                    .physics_objects
                    .collider_set
                    .get_mut(*collider)
                {
                    collider.set_density(0.0);
                }
            }
            body.set_additional_mass_properties(
                MassProperties::new(Point { coords: local_com }, mass, inertia),
                wake_up,
            );
            if force_update {
                body.recompute_mass_properties_from_colliders(
                    &physics_world.physics_objects.collider_set,
                );
            }
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_add_force(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        force: Vector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.add_force(force, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_add_force_at_point(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        force: Vector<Real>,
        point: Vector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            let local_point = Point { coords: point } + body.center_of_mass().coords;
            body.add_force_at_point(force, local_point, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_add_torque(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        torque: AngVector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.add_torque(torque, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_apply_impulse(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        impulse: Vector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.apply_impulse(impulse, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_apply_impulse_at_point(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        impulse: Vector<Real>,
        point: Vector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            let mut local_point = Point { coords: point };
            local_point += body.center_of_mass().coords;
            body.apply_impulse_at_point(impulse, local_point, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_get_constant_force(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) -> Vector<Real> {
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get(body_handle)
        {
            return body.user_force();
        }
        Vector::default()
    }

    pub fn body_get_constant_torque(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
    ) -> AngVector<Real> {
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get(body_handle)
        {
            return body.user_torque();
        }
        ANG_ZERO
    }

    pub fn body_apply_torque_impulse(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        torque_impulse: AngVector<Real>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.apply_torque_impulse(torque_impulse, true);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_reset_torques(&mut self, world_handle: WorldHandle, body_handle: RigidBodyHandle) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.reset_torques(false);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_reset_forces(&mut self, world_handle: WorldHandle, body_handle: RigidBodyHandle) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.reset_forces(false);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_wake_up(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        strong: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
            && body.is_sleeping()
        {
            body.wake_up(strong);
        }
        self.body_wake_up_connected_rigidbodies(world_handle, body_handle);
    }

    pub fn body_force_sleep(&mut self, world_handle: WorldHandle, body_handle: RigidBodyHandle) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            body.sleep();
        }
    }

    pub fn body_get_mass_properties(
        &mut self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        mass: Real,
    ) -> RigidBodyMassProps {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(body_handle)
        {
            for collider in body.colliders() {
                if let Some(collider) = physics_world
                    .physics_objects
                    .collider_set
                    .get_mut(*collider)
                {
                    collider.set_mass(mass);
                }
            }
            body.recompute_mass_properties_from_colliders(
                &physics_world.physics_objects.collider_set,
            );
            return body.mass_properties().clone();
        }
        RigidBodyMassProps::default()
    }

    pub fn body_get_colliders(
        &mut self,
        world_handle: WorldHandle,
        rigidbody_handle: RigidBodyHandle,
    ) -> &[ColliderHandle] {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(body) = physics_world
                .physics_objects
                .rigid_body_set
                .get_mut(rigidbody_handle)
        {
            return body.colliders();
        }
        &[]
    }
}
