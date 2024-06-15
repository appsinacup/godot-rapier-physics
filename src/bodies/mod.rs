pub mod rapier_area;
pub mod rapier_body;
pub mod rapier_collision_object;
#[cfg(feature = "dim2")]
pub mod rapier_direct_body_state_2d;
#[cfg(feature = "dim3")]
pub mod rapier_direct_body_state_3d;
#[cfg(feature = "dim2")]
pub type RapierDirectBodyState = rapier_direct_body_state_2d::RapierDirectBodyState2D;
#[cfg(feature = "dim2")]
pub type PhysicsDirectBodyState = godot::engine::PhysicsDirectBodyState2D;
#[cfg(feature = "dim3")]
pub type RapierDirectBodyState = rapier_direct_body_state_3d::RapierDirectBodyState3D;
#[cfg(feature = "dim3")]
pub type PhysicsDirectBodyState = godot::engine::PhysicsDirectBodyState3D;

#[cfg(feature = "dim3")]
pub fn transform_scale(transform : &godot::builtin::Transform3D) -> godot::builtin::Vector3 {
    return transform.basis.scale()
}
#[cfg(feature = "dim2")]
pub fn transform_scale(transform : &godot::builtin::Transform2D) -> godot::builtin::Vector2 {
    return transform.scale()
}
#[cfg(feature = "dim3")]
pub fn transform_rotation(transform : &godot::builtin::Transform3D) -> godot::builtin::Vector3 {
    return transform.basis.to_euler(godot::builtin::EulerOrder::XYZ)
}
#[cfg(feature = "dim2")]
pub fn transform_rotation(transform : &godot::builtin::Transform2D) -> real {
    return transform.rotation()
}
#[cfg(feature = "dim3")]
pub fn transform_rotation_rapier(transform : &godot::builtin::Transform3D) -> rapier::math::AngVector<rapier::math::Real> {

    use crate::rapier_wrapper::convert::vector_to_rapier;

    return vector_to_rapier(transform.basis.to_euler(godot::builtin::EulerOrder::XYZ))
}
#[cfg(feature = "dim2")]
pub fn transform_rotation_rapier(transform : &godot::builtin::Transform2D) -> real {
    return transform.rotation()
}