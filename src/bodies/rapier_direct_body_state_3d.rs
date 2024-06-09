use crate::{
    bodies::rapier_collision_object::IRapierCollisionObject,
    servers::rapier_physics_singleton::{bodies_singleton, spaces_singleton},
    spaces::rapier_space::RapierSpace,
    Vector,
};
use godot::{
    engine::{
        physics_server_3d, IPhysicsDirectBodyState3DExtension, PhysicsDirectBodyState3DExtension,
        PhysicsDirectSpaceState2D, PhysicsDirectSpaceState3D,
    },
    prelude::*,
};

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState3DExtension,tool)]
pub struct RapierDirectBodyState3D {
    pub body: Rid,

    base: Base<PhysicsDirectBodyState3DExtension>,
}

impl RapierDirectBodyState3D {
    pub fn set_body(&mut self, body: Rid) {
        self.body = body;
    }
}

#[godot_api]
impl IPhysicsDirectBodyState3DExtension for RapierDirectBodyState3D {
    fn init(base: Base<PhysicsDirectBodyState3DExtension>) -> Self {
        Self {
            body: Rid::Invalid,
            base,
        }
    }

    fn get_total_gravity(&self) -> Vector3 {
        let mut space_rid = Rid::Invalid;
        let mut gravity_scale = 1.0;
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            space_rid = body.get_base().get_space();
            if let Some(body) = body.get_body() {
                if body.using_area_gravity() {
                    return body.total_gravity();
                } else {
                    gravity_scale = body.gravity_scale();
                }
            }
        }
        if let Some(space) = spaces_singleton().spaces.get(&space_rid) {
            let default_gravity: f32 = space
                .get_default_area_param(physics_server_3d::AreaParameter::GRAVITY)
                .to();
            let default_gravity_vector: Vector = space
                .get_default_area_param(physics_server_3d::AreaParameter::GRAVITY_VECTOR)
                .to();
            return default_gravity_vector * gravity_scale * default_gravity;
        }
        Vector3::ZERO
    }

    fn get_total_linear_damp(&self) -> f32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.total_linear_damping();
            }
        }
        0.0
    }

    fn get_total_angular_damp(&self) -> f32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.total_angular_damping();
            }
        }
        0.0
    }

    fn get_center_of_mass(&self) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            let base = body.get_base();
            if let Some(body) = body.get_body() {
                return base.get_transform() * body.get_center_of_mass();
            }
        }
        Vector::ZERO
    }

    fn get_center_of_mass_local(&self) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_center_of_mass();
            }
        }
        Vector::ZERO
    }

    fn get_inverse_mass(&self) -> f32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_inv_mass();
            }
        }
        0.0
    }

    fn get_inverse_inertia(&self) -> Vector3 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_inv_inertia();
            }
        }
        Vector3::ZERO
    }

    fn set_linear_velocity(&mut self, velocity: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_linear_velocity(velocity);
            }
        }
    }

    fn get_linear_velocity(&self) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_linear_velocity();
            }
        }
        Vector::ZERO
    }

    fn set_angular_velocity(&mut self, velocity: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_angular_velocity(velocity);
            }
        }
    }

    fn get_angular_velocity(&self) -> Vector3 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_angular_velocity();
            }
        }
        Vector3::ZERO
    }

    fn set_transform(&mut self, transform: Transform3D) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.get_mut_base().set_transform(transform, true);
            }
        }
    }

    fn get_transform(&self) -> Transform3D {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_base().get_transform();
            }
        }
        Transform3D::default()
    }

    fn get_velocity_at_local_position(&self, local_position: Vector) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_velocity_at_local_point(local_position);
            }
        }
        Vector::default()
    }

    fn apply_central_impulse(&mut self, impulse: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_central_impulse(impulse)
            }
        }
    }

    fn apply_impulse(&mut self, impulse: Vector, position: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_impulse(impulse, position)
            }
        }
    }

    fn apply_torque_impulse(&mut self, impulse: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_torque_impulse(impulse)
            }
        }
    }

    fn apply_central_force(&mut self, force: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_central_force(force)
            }
        }
    }

    fn apply_force(&mut self, force: Vector, position: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_force(force, position)
            }
        }
    }

    fn apply_torque(&mut self, torque: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_torque(torque)
            }
        }
    }

    fn add_constant_central_force(&mut self, force: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.add_constant_central_force(force)
            }
        }
    }

    fn add_constant_force(&mut self, force: Vector, position: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.add_constant_force(force, position)
            }
        }
    }

    fn add_constant_torque(&mut self, torque: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.add_constant_torque(torque)
            }
        }
    }

    fn set_constant_force(&mut self, force: Vector) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_constant_force(force)
            }
        }
    }

    fn get_constant_force(&self) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_force();
            }
        }
        Vector::default()
    }

    fn set_constant_torque(&mut self, torque: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_constant_torque(torque)
            }
        }
    }

    fn get_constant_torque(&self) -> Vector3 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_torque();
            }
        }
        Vector3::ZERO
    }

    fn set_sleep_state(&mut self, enabled: bool) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                if let Some(space) = spaces_singleton()
                    .spaces
                    .get_mut(&body.get_base().get_space())
                {
                    body.set_active(!enabled, space);
                }
            }
        }
    }

    fn is_sleeping(&self) -> bool {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return !body.is_active();
            }
        }
        false
    }

    fn get_contact_count(&self) -> i32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contact_count();
            }
        }
        0
    }

    fn get_contact_local_position(&self, contact_idx: i32) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_pos;
                }
            }
        }
        Vector::default()
    }

    fn get_contact_local_normal(&self, contact_idx: i32) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_normal;
                }
            }
        }
        Vector::default()
    }

    fn get_contact_local_shape(&self, contact_idx: i32) -> i32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_shape;
                }
            }
        }
        0
    }

    fn get_contact_local_velocity_at_position(&self, contact_idx: i32) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.local_velocity_at_pos;
                }
            }
        }
        Vector::default()
    }

    fn get_contact_collider(&self, contact_idx: i32) -> Rid {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider;
                }
            }
        }
        Rid::Invalid
    }

    fn get_contact_collider_position(&self, contact_idx: i32) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_pos;
                }
            }
        }
        Vector::default()
    }

    fn get_contact_collider_id(&self, contact_idx: i32) -> u64 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_instance_id;
                }
            }
        }
        0
    }

    fn get_contact_collider_object(&self, contact_idx: i32) -> Option<Gd<Object>> {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
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

    fn get_contact_collider_shape(&self, contact_idx: i32) -> i32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_shape;
                }
            }
        }
        0
    }

    fn get_contact_collider_velocity_at_position(&self, contact_idx: i32) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.collider_velocity_at_pos;
                }
            }
        }
        Vector::default()
    }

    fn get_contact_impulse(&self, contact_idx: i32) -> Vector {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                if let Some(contact) = body.contacts().get(contact_idx as usize) {
                    return contact.impulse;
                }
            }
        }
        Vector::default()
    }

    fn get_step(&self) -> f32 {
        RapierSpace::get_last_step()
    }

    fn integrate_forces(&mut self) {}

    fn get_space_state(&mut self) -> Option<Gd<PhysicsDirectSpaceState3D>> {
        if let Some(body) = bodies_singleton().collision_objects.get(&self.body) {
            if let Some(space) = spaces_singleton().spaces.get(&body.get_base().get_space()) {
                return space.get_direct_state().clone();
            }
        }
        None
    }
}
