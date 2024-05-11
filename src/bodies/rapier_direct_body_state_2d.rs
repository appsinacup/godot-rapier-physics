use crate::{
    bodies::rapier_collision_object_2d::IRapierCollisionObject2D,
    servers::rapier_physics_singleton_2d::{bodies_singleton, spaces_singleton},
};
use godot::{
    engine::{
        physics_server_2d, IPhysicsDirectBodyState2DExtension, PhysicsDirectBodyState2DExtension,
        PhysicsDirectSpaceState2D,
    },
    prelude::*,
};

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState2DExtension)]
pub struct RapierDirectBodyState2D {
    body: Rid,

    base: Base<PhysicsDirectBodyState2DExtension>,
}

#[godot_api]
impl RapierDirectBodyState2D {
    #[func]
    pub fn set_body(&mut self, body: Rid) {
        self.body = body;
    }
}

#[godot_api]
impl IPhysicsDirectBodyState2DExtension for RapierDirectBodyState2D {
    fn init(base: Base<PhysicsDirectBodyState2DExtension>) -> Self {
        Self {
            body: Rid::Invalid,
            base,
        }
    }

    fn get_total_gravity(&self) -> Vector2 {
        let mut space_rid = Rid::Invalid;
        let mut gravity_scale = 1.0;
        {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&self.body) {
                space_rid = body.get_base().get_space();
                if let Some(body) = body.get_body() {
                    if body.using_area_gravity {
                        return body.total_gravity;
                    } else {
                        gravity_scale = body.gravity_scale;
                    }
                }
            }
        }
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&space_rid) {
            let default_gravity: f32 = space
                .get_default_area_param(physics_server_2d::AreaParameter::GRAVITY)
                .to();
            let default_gravity_vector: Vector2 = space
                .get_default_area_param(physics_server_2d::AreaParameter::GRAVITY_VECTOR)
                .to();
            return default_gravity_vector * gravity_scale * default_gravity;
        }
        Vector2::ZERO
    }

    fn get_total_linear_damp(&self) -> f32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.total_linear_damping;
            }
        }
        0.0
    }

    fn get_total_angular_damp(&self) -> f32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.total_angular_damping;
            }
        }
        0.0
    }

    fn get_center_of_mass(&self) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            let base = body.get_base();
            if let Some(body) = body.get_body() {
                return base.get_transform().basis_xform(body.get_center_of_mass());
            }
        }
        Vector2::ZERO
    }

    fn get_center_of_mass_local(&self) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_center_of_mass();
            }
        }
        Vector2::ZERO
    }

    fn get_inverse_mass(&self) -> f32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_inv_mass();
            }
        }
        0.0
    }

    fn get_inverse_inertia(&self) -> f32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_inv_inertia();
            }
        }
        0.0
    }

    fn set_linear_velocity(&mut self, velocity: Vector2) {
        let mut lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_linear_velocity(velocity);
            }
        }
    }

    fn get_linear_velocity(&self) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_linear_velocity();
            }
        }
        Vector2::ZERO
    }

    fn set_angular_velocity(&mut self, velocity: f32) {
        let mut lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.set_angular_velocity(velocity);
            }
        }
    }

    fn get_angular_velocity(&self) -> f32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_angular_velocity();
            }
        }
        0.0
    }

    fn set_transform(&mut self, transform: Transform2D) {
        let mut lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get_mut(&self.body) {
            if let Some(body) = body.get_mut_body() {
                body.get_mut_base().set_transform(transform, true);
            }
        }
    }

    fn get_transform(&self) -> Transform2D {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_base().get_transform();
            }
        }
        Transform2D::default()
    }

    fn get_velocity_at_local_position(&self, local_position: Vector2) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_velocity_at_local_point(local_position);
            }
        }
        Vector2::default()
    }

    fn apply_central_impulse(&mut self, impulse: Vector2) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.apply_central_impulse(impulse);
            }
        }
    }

    fn apply_impulse(&mut self, impulse: Vector2, position: Vector2) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.apply_impulse(impulse, position);
            }
        }
    }

    fn apply_torque_impulse(&mut self, impulse: f32) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.apply_torque_impulse(impulse);
            }
        }
    }

    fn apply_central_force(&mut self, force: Vector2) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.apply_central_force(force);
            }
        }
    }

    fn apply_force(&mut self, force: Vector2, position: Vector2) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.apply_force(force, position);
            }
        }
    }

    fn apply_torque(&mut self, torque: f32) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.apply_torque(torque);
            }
        }
    }

    fn add_constant_central_force(&mut self, force: Vector2) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.add_constant_central_force(force);
            }
        }
    }

    fn add_constant_force(&mut self, force: Vector2, position: Vector2) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.add_constant_force(force, position);
            }
        }
    }

    fn add_constant_torque(&mut self, torque: f32) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.add_constant_torque(torque);
            }
        }
    }

    fn set_constant_force(&mut self, force: Vector2) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.set_constant_force(force);
            }
        }
    }

    fn get_constant_force(&self) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_force();
            }
        }
        Vector2::default()
    }

    fn set_constant_torque(&mut self, torque: f32) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.set_constant_torque(torque);
            }
        }
    }

    fn get_constant_torque(&self) -> f32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_torque();
            }
        }
        0.0
    }

    fn set_sleep_state(&mut self, enabled: bool) {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.set_active(!enabled);
            }
        }
    }

    fn is_sleeping(&self) -> bool {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return !body.is_active();
            }
        }
        false
    }

    fn get_contact_count(&self) -> i32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contact_count;
            }
        }
        0
    }

    fn get_contact_local_position(&self, contact_idx: i32) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].local_pos;
            }
        }
        Vector2::default()
    }

    fn get_contact_local_normal(&self, contact_idx: i32) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].local_normal;
            }
        }
        Vector2::default()
    }

    fn get_contact_local_shape(&self, contact_idx: i32) -> i32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].local_shape;
            }
        }
        0
    }

    fn get_contact_local_velocity_at_position(&self, contact_idx: i32) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].local_velocity_at_pos;
            }
        }
        Vector2::default()
    }

    fn get_contact_collider(&self, contact_idx: i32) -> Rid {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].collider;
            }
        }
        Rid::Invalid
    }

    fn get_contact_collider_position(&self, contact_idx: i32) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].collider_pos;
            }
        }
        Vector2::default()
    }

    fn get_contact_collider_id(&self, contact_idx: i32) -> u64 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize]
                    .collider_instance_id
            }
        }
        0
    }

    fn get_contact_collider_object(&self, contact_idx: i32) -> Option<Gd<Object>> {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return Some(Gd::from_instance_id(
                    InstanceId::from_i64(body.contacts[contact_idx as usize].collider_instance_id as i64),
                ));
            }
        }
        None
    }

    fn get_contact_collider_shape(&self, contact_idx: i32) -> i32 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].collider_shape;
            }
        }
        0
    }

    fn get_contact_collider_velocity_at_position(&self, contact_idx: i32) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].collider_velocity_at_pos;
            }
        }
        Vector2::default()
    }

    fn get_contact_impulse(&self, contact_idx: i32) -> Vector2 {
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get(&self.body) {
            if let Some(body) = body.get_body() {
                return body.contacts[contact_idx as usize].impulse;
            }
        }
        Vector2::default()
    }

    fn get_step(&self) -> f32 {
        let mut space_rid = Rid::Invalid;
        {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&self.body) {
                space_rid = body.get_base().get_space();
            }
        }
        {
            let lock = spaces_singleton().lock().unwrap();
            if let Some(space) = lock.spaces.get(&space_rid) {
                return space.get_last_step();
            }
        }
        0.0
    }

    fn integrate_forces(&mut self) {}

    fn get_space_state(&mut self) -> Option<Gd<PhysicsDirectSpaceState2D>> {
        let mut space_rid = Rid::Invalid;
        {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&self.body) {
                space_rid = body.get_base().get_space();
            }
        }
        {
            let lock = spaces_singleton().lock().unwrap();
            if let Some(space) = lock.spaces.get(&space_rid) {
                return space.get_direct_state();
            }
        }
        None
    }
}
