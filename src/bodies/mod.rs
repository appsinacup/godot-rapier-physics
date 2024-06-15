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
pub fn transform_scale(transform: &crate::Transform) -> crate::Vector {
    transform.basis.scale()
}
#[cfg(feature = "dim2")]
pub fn transform_scale(transform: &crate::Transform) -> crate::Vector {
    transform.scale()
}
#[cfg(feature = "dim3")]
pub fn transform_rotation(transform: &crate::Transform) -> crate::Vector {
    transform.basis.to_euler(godot::builtin::EulerOrder::XYZ)
}
#[cfg(feature = "dim2")]
pub fn transform_update(
    transform: &crate::Transform,
    rotation: crate::Angle,
    origin: crate::Vector,
) -> crate::Transform {
    use crate::Transform;
    Transform::from_angle_scale_skew_origin(rotation, transform.scale(), transform.skew(), origin)
}
#[cfg(feature = "dim3")]
pub fn transform_update(
    transform: &crate::Transform,
    rotation: crate::Angle,
    origin: crate::Vector,
) -> crate::Transform {
    use godot::builtin::Basis;
    use godot::builtin::EulerOrder;

    use crate::Transform;
    let new_transform = Transform::new(Basis::from_euler(EulerOrder::XYZ, rotation), origin);
    let scale = transform.basis.scale();
    new_transform.scaled_local(scale)
}
#[cfg(feature = "dim2")]
pub fn transform_rotation(
    transform: &godot::builtin::Transform2D,
) -> rapier::math::AngVector<rapier::math::Real> {
    transform.rotation()
}
#[cfg(feature = "dim3")]
pub fn transform_rotation_rapier(
    transform: &godot::builtin::Transform3D,
) -> rapier::math::AngVector<rapier::math::Real> {
    use crate::rapier_wrapper::convert::vector_to_rapier;
    vector_to_rapier(transform.basis.to_euler(godot::builtin::EulerOrder::XYZ))
}
#[cfg(feature = "dim2")]
pub fn transform_rotation_rapier(
    transform: &godot::builtin::Transform2D,
) -> rapier::math::AngVector<rapier::math::Real> {
    transform.rotation()
}
