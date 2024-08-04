use godot::classes::*;
use godot::prelude::*;
#[cfg(feature = "dim2")]
use physics_server_2d::*;
#[cfg(feature = "dim3")]
use physics_server_3d::*;

use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::servers::rapier_physics_singleton::physics_data;
use crate::types::*;
pub struct RapierDirectBodyStateImpl {
    body: Rid,
}
impl RapierDirectBodyStateImpl {
    pub(super) fn set_body(&mut self, body: Rid) {
        self.body = body;
    }

    pub(super) fn get_body(&self) -> &Rid {
        &self.body
    }

    pub(super) fn default() -> Self {
        Self { body: Rid::Invalid }
    }

    pub(super) fn get_total_gravity(&self) -> Vector {
        let mut space_rid = Rid::Invalid;
        let mut gravity_scale = 1.0;
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            space_rid = body.get_base().get_space();
            if let Some(body) = body.get_body() {
                if body.using_area_gravity() {
                    return body.total_gravity();
                } else {
                    gravity_scale = body.gravity_scale();
                }
            }
        }
        if let Some(space) = physics_data.spaces.get(&space_rid) {
            let default_gravity =
                variant_to_float(&space.get_default_area_param(AreaParameter::GRAVITY));
            let default_gravity_vector: Vector = space
                .get_default_area_param(AreaParameter::GRAVITY_VECTOR)
                .try_to()
                .unwrap_or_default();
            return default_gravity_vector * gravity_scale * default_gravity;
        }
        Vector::ZERO
    }

    pub(super) fn get_total_linear_damp(&self) -> f32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.total_linear_damping();
            }
        }
        0.0
    }

    pub(super) fn get_total_angular_damp(&self) -> f32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.total_angular_damping();
            }
        }
        0.0
    }

    pub(super) fn get_center_of_mass(&self) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            let base = body.get_base();
            if let Some(body) = body.get_body() {
                return base.get_transform() * body.get_center_of_mass();
            }
        }
        Vector::ZERO
    }

    pub(super) fn get_center_of_mass_local(&self) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_center_of_mass();
            }
        }
        Vector::ZERO
    }

    pub(super) fn get_inverse_mass(&self) -> f32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_inv_mass();
            }
        }
        0.0
    }

    pub(super) fn get_inverse_inertia(&self) -> Angle {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_inv_inertia();
            }
        }
        ANGLE_ZERO
    }

    #[cfg(feature = "dim3")]
    pub(super) fn get_inverse_inertia_tensor(&self) -> Basis {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_inv_inertia_tensor();
            }
        }
        Basis::IDENTITY
    }

    #[cfg(feature = "dim3")]
    pub(super) fn get_principal_inertia_axes(&self) -> Basis {
        // TODO
        Basis::IDENTITY
    }

    pub(super) fn set_linear_velocity(&mut self, velocity: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_linear_velocity(velocity, &mut physics_data.physics_engine);
            }
        }
    }

    pub(super) fn get_linear_velocity(&self) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_linear_velocity(&physics_data.physics_engine);
            }
        }
        Vector::ZERO
    }

    pub(super) fn set_angular_velocity(&mut self, velocity: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_angular_velocity(velocity, &mut physics_data.physics_engine);
            }
        }
    }

    pub(super) fn get_angular_velocity(&self) -> Angle {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_angular_velocity(&physics_data.physics_engine);
            }
        }
        ANGLE_ZERO
    }

    pub(super) fn set_transform(&mut self, transform: Transform) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.get_mut_base().set_transform(
                    transform,
                    true,
                    &mut physics_data.physics_engine,
                );
            }
        }
    }

    pub(super) fn get_transform(&self) -> Transform {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_base().get_transform();
            }
        }
        Transform::IDENTITY
    }

    pub(super) fn get_velocity_at_local_position(&self, local_position: Vector) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body
                    .get_velocity_at_local_point(local_position, &physics_data.physics_engine);
            }
        }
        Vector::default()
    }

    pub(super) fn apply_central_impulse(&mut self, impulse: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.apply_central_impulse(impulse, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn apply_impulse(&mut self, impulse: Vector, position: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.apply_impulse(impulse, position, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn apply_torque_impulse(&mut self, impulse: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.apply_torque_impulse(impulse, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn apply_central_force(&mut self, force: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.apply_central_force(force, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn apply_force(&mut self, force: Vector, position: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.apply_force(force, position, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn apply_torque(&mut self, torque: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.apply_torque(torque, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn add_constant_central_force(&mut self, force: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.add_constant_central_force(force, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn add_constant_force(&mut self, force: Vector, position: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.add_constant_force(force, position, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn add_constant_torque(&mut self, torque: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.add_constant_torque(torque, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn set_constant_force(&mut self, force: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.set_constant_force(force, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn get_constant_force(&self) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_force(&physics_data.physics_engine);
            }
        }
        Vector::default()
    }

    pub(super) fn set_constant_torque(&mut self, torque: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.force_mass_update(&mut physics_data.spaces, &mut physics_data.physics_engine);
                body.set_constant_torque(torque, &mut physics_data.physics_engine)
            }
        }
    }

    pub(super) fn get_constant_torque(&self) -> Angle {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_torque(&physics_data.physics_engine);
            }
        }
        ANGLE_ZERO
    }

    pub(super) fn set_sleep_state(&mut self, enabled: bool) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                if let Some(space) = physics_data.spaces.get_mut(&body.get_base().get_space()) {
                    body.set_active(!enabled, space);
                }
            }
        }
    }

    pub(super) fn is_sleeping(&self) -> bool {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return !body.is_active();
            }
        }
        false
    }

    pub(super) fn get_contact_count(&self) -> i32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contact_count();
            }
        }
        0
    }

    pub(super) fn get_contact_local_position(&self, contact_idx: i32) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_pos;
                }
            }
        }
        Vector::default()
    }

    pub(super) fn get_contact_local_normal(&self, contact_idx: i32) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_normal;
                }
            }
        }
        Vector::default()
    }

    pub(super) fn get_contact_local_shape(&self, contact_idx: i32) -> i32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_shape;
                }
            }
        }
        0
    }

    pub(super) fn get_contact_local_velocity_at_position(&self, contact_idx: i32) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_velocity_at_pos;
                }
            }
        }
        Vector::default()
    }

    pub(super) fn get_contact_collider(&self, contact_idx: i32) -> Rid {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider;
                }
            }
        }
        Rid::Invalid
    }

    pub(super) fn get_contact_collider_position(&self, contact_idx: i32) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_pos;
                }
            }
        }
        Vector::default()
    }

    pub(super) fn get_contact_collider_id(&self, contact_idx: i32) -> u64 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_instance_id;
                }
            }
        }
        0
    }

    pub(super) fn get_contact_collider_object(&self, contact_idx: i32) -> Option<Gd<Object>> {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return Some(Gd::from_instance_id(InstanceId::from_i64(
                        contact.collider_instance_id as i64,
                    )));
                }
            }
        }
        None
    }

    pub(super) fn get_contact_collider_shape(&self, contact_idx: i32) -> i32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_shape;
                }
            }
        }
        0
    }

    pub(super) fn get_contact_collider_velocity_at_position(&self, contact_idx: i32) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_velocity_at_pos;
                }
            }
        }
        Vector::default()
    }

    pub(super) fn get_contact_impulse(&self, contact_idx: i32) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.impulse;
                }
            }
        }
        Vector::default()
    }
}
