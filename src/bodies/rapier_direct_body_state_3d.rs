use godot::classes::*;
use godot::prelude::*;

use super::rapier_direct_body_state_impl::RapierDirectBodyStateImpl;
use crate::servers::rapier_physics_singleton::physics_data;
use crate::spaces::rapier_space::RapierSpace;
use crate::types::*;
#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState3DExtension,tool)]
pub struct RapierDirectBodyState3D {
    implementation: RapierDirectBodyStateImpl,
    base: Base<PhysicsDirectBodyState3DExtension>,
}
impl RapierDirectBodyState3D {
    pub fn set_body(&mut self, body: Rid) {
        self.implementation.set_body(body);
    }
}
#[godot_api]
impl IPhysicsDirectBodyState3DExtension for RapierDirectBodyState3D {
    fn init(base: Base<PhysicsDirectBodyState3DExtension>) -> Self {
        Self {
            implementation: RapierDirectBodyStateImpl::default(),
            base,
        }
    }

    fn get_total_gravity(&self) -> Vector {
        self.implementation.get_total_gravity()
    }

    fn get_total_linear_damp(&self) -> f32 {
        self.implementation.get_total_linear_damp()
    }

    fn get_total_angular_damp(&self) -> f32 {
        self.implementation.get_total_angular_damp()
    }

    fn get_center_of_mass(&self) -> Vector {
        self.implementation.get_center_of_mass()
    }

    fn get_center_of_mass_local(&self) -> Vector {
        self.implementation.get_center_of_mass_local()
    }

    fn get_inverse_mass(&self) -> f32 {
        self.implementation.get_inverse_mass()
    }

    fn get_inverse_inertia(&self) -> Angle {
        self.implementation.get_inverse_inertia()
    }

    fn get_inverse_inertia_tensor(&self) -> Basis {
        self.implementation.get_inverse_inertia_tensor()
    }

    fn get_principal_inertia_axes(&self) -> Basis {
        self.implementation.get_principal_inertia_axes()
    }

    fn set_linear_velocity(&mut self, velocity: Vector) {
        self.implementation.set_linear_velocity(velocity)
    }

    fn get_linear_velocity(&self) -> Vector {
        self.implementation.get_linear_velocity()
    }

    fn set_angular_velocity(&mut self, velocity: Angle) {
        self.implementation.set_angular_velocity(velocity)
    }

    fn get_angular_velocity(&self) -> Angle {
        self.implementation.get_angular_velocity()
    }

    fn set_transform(&mut self, transform: Transform) {
        self.implementation.set_transform(transform)
    }

    fn get_transform(&self) -> Transform {
        self.implementation.get_transform()
    }

    fn get_velocity_at_local_position(&self, local_position: Vector) -> Vector {
        self.implementation
            .get_velocity_at_local_position(local_position)
    }

    fn apply_central_impulse(&mut self, impulse: Vector) {
        self.implementation.apply_central_impulse(impulse)
    }

    fn apply_impulse(&mut self, impulse: Vector, position: Vector) {
        self.implementation.apply_impulse(impulse, position)
    }

    fn apply_torque_impulse(&mut self, impulse: Angle) {
        self.implementation.apply_torque_impulse(impulse)
    }

    fn apply_central_force(&mut self, force: Vector) {
        self.implementation.apply_central_force(force)
    }

    fn apply_force(&mut self, force: Vector, position: Vector) {
        self.implementation.apply_force(force, position)
    }

    fn apply_torque(&mut self, torque: Angle) {
        self.implementation.apply_torque(torque)
    }

    fn add_constant_central_force(&mut self, force: Vector) {
        self.implementation.add_constant_central_force(force)
    }

    fn add_constant_force(&mut self, force: Vector, position: Vector) {
        self.implementation.add_constant_force(force, position)
    }

    fn add_constant_torque(&mut self, torque: Angle) {
        self.implementation.add_constant_torque(torque)
    }

    fn set_constant_force(&mut self, force: Vector) {
        self.implementation.set_constant_force(force)
    }

    fn get_constant_force(&self) -> Vector {
        self.implementation.get_constant_force()
    }

    fn set_constant_torque(&mut self, torque: Angle) {
        self.implementation.set_constant_torque(torque)
    }

    fn get_constant_torque(&self) -> Angle {
        self.implementation.get_constant_torque()
    }

    fn set_sleep_state(&mut self, enabled: bool) {
        self.implementation.set_sleep_state(enabled)
    }

    fn is_sleeping(&self) -> bool {
        self.implementation.is_sleeping()
    }

    fn get_contact_count(&self) -> i32 {
        self.implementation.get_contact_count()
    }

    fn get_contact_local_position(&self, contact_idx: i32) -> Vector {
        self.implementation.get_contact_local_position(contact_idx)
    }

    fn get_contact_local_normal(&self, contact_idx: i32) -> Vector {
        self.implementation.get_contact_local_normal(contact_idx)
    }

    fn get_contact_local_shape(&self, contact_idx: i32) -> i32 {
        self.implementation.get_contact_local_shape(contact_idx)
    }

    fn get_contact_local_velocity_at_position(&self, contact_idx: i32) -> Vector {
        self.implementation
            .get_contact_local_velocity_at_position(contact_idx)
    }

    fn get_contact_collider(&self, contact_idx: i32) -> Rid {
        self.implementation.get_contact_collider(contact_idx)
    }

    fn get_contact_collider_position(&self, contact_idx: i32) -> Vector {
        self.implementation
            .get_contact_collider_position(contact_idx)
    }

    fn get_contact_collider_id(&self, contact_idx: i32) -> u64 {
        self.implementation.get_contact_collider_id(contact_idx)
    }

    fn get_contact_collider_object(&self, contact_idx: i32) -> Option<Gd<Object>> {
        self.implementation.get_contact_collider_object(contact_idx)
    }

    fn get_contact_collider_shape(&self, contact_idx: i32) -> i32 {
        self.implementation.get_contact_collider_shape(contact_idx)
    }

    fn get_contact_collider_velocity_at_position(&self, contact_idx: i32) -> Vector {
        self.implementation
            .get_contact_collider_velocity_at_position(contact_idx)
    }

    fn get_contact_impulse(&self, contact_idx: i32) -> Vector {
        self.implementation.get_contact_impulse(contact_idx)
    }

    fn get_step(&self) -> f32 {
        RapierSpace::get_last_step()
    }

    fn integrate_forces(&mut self) {}

    fn get_space_state(&mut self) -> Option<Gd<PhysicsDirectSpaceState3D>> {
        let physics_data = physics_data();
        if let Some(body) = physics_data
            .collision_objects
            .get(self.implementation.get_body())
        {
            if let Some(space) = physics_data.spaces.get(&body.get_base().get_space()) {
                return space.get_direct_state().clone();
            }
        }
        None
    }
}
