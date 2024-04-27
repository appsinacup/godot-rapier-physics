use godot::{engine::{IPhysicsDirectBodyState2DExtension, PhysicsDirectBodyState2DExtension, PhysicsDirectSpaceState2D}, prelude::*};

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState2DExtension)]
pub struct RapierDirectBodyState2D {
    body: Rid,

    base: Base<PhysicsDirectBodyState2DExtension>,
}


#[godot_api]
impl IPhysicsDirectBodyState2DExtension for RapierDirectBodyState2D {
    fn init(base: Base<PhysicsDirectBodyState2DExtension>) -> Self {
        Self {
            body: Rid::Invalid,
            base,
        }
    }
    
    fn get_total_gravity(&self,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_total_linear_damp(&self,) -> f32 {
        0.0
    }
    
    fn get_total_angular_damp(&self,) -> f32 {
        0.0
    }
    
    fn get_center_of_mass(&self,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_center_of_mass_local(&self,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_inverse_mass(&self,) -> f32 {
        0.0
    }
    
    fn get_inverse_inertia(&self,) -> f32 {
        0.0
    }
    
    fn set_linear_velocity(&mut self, velocity: Vector2,) {
    }
    
    fn get_linear_velocity(&self,) -> Vector2 {
        Vector2::default()
    }
    
    fn set_angular_velocity(&mut self, velocity: f32,) {
    }
    
    fn get_angular_velocity(&self,) -> f32 {
        0.0
    }
    
    fn set_transform(&mut self, transform: Transform2D,) {
    }
    
    fn get_transform(&self,) -> Transform2D {
        Transform2D::default()
    }
    
    fn get_velocity_at_local_position(&self, local_position: Vector2,) -> Vector2 {
        Vector2::default()
    }
    
    fn apply_central_impulse(&mut self, impulse: Vector2,) {
    }
    
    fn apply_impulse(&mut self, impulse: Vector2, position: Vector2,) {
    }
    
    fn apply_torque_impulse(&mut self, impulse: f32,) {
    }
    
    fn apply_central_force(&mut self, force: Vector2,) {
    }
    
    fn apply_force(&mut self, force: Vector2, position: Vector2,) {
    }
    
    fn apply_torque(&mut self, torque: f32,) {
    }
    
    fn add_constant_central_force(&mut self, force: Vector2,) {
    }
    
    fn add_constant_force(&mut self, force: Vector2, position: Vector2,) {
    }
    
    fn add_constant_torque(&mut self, torque: f32,) {
    }
    
    fn set_constant_force(&mut self, force: Vector2,) {
    }
    
    fn get_constant_force(&self,) -> Vector2 {
        Vector2::default()
    }
    
    fn set_constant_torque(&mut self, torque: f32,) {
    }
    
    fn get_constant_torque(&self,) -> f32 {
        0.0
    }
    
    fn set_sleep_state(&mut self, enabled: bool,) {
    }
    
    fn is_sleeping(&self,) -> bool {
        false
    }
    
    fn get_contact_count(&self,) -> i32 {
        0
    }
    
    fn get_contact_local_position(&self, contact_idx: i32,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_contact_local_normal(&self, contact_idx: i32,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_contact_local_shape(&self, contact_idx: i32,) -> i32 {
        0
    }
    
    fn get_contact_local_velocity_at_position(&self, contact_idx: i32,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_contact_collider(&self, contact_idx: i32,) -> Rid {
        Rid::Invalid
    }
    
    fn get_contact_collider_position(&self, contact_idx: i32,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_contact_collider_id(&self, contact_idx: i32,) -> u64 {
        0
    }
    
    fn get_contact_collider_object(&self, contact_idx: i32,) -> Option< Gd< Object > > {
        None
    }
    
    fn get_contact_collider_shape(&self, contact_idx: i32,) -> i32 {
        0
    }
    
    fn get_contact_collider_velocity_at_position(&self, contact_idx: i32,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_contact_impulse(&self, contact_idx: i32,) -> Vector2 {
        Vector2::default()
    }
    
    fn get_step(&self,) -> f32 {
        0.0
    }
    
    fn integrate_forces(&mut self,) {
    }
    
    fn get_space_state(&mut self,) -> Option< Gd< PhysicsDirectSpaceState2D > > {
        None
    }

}
