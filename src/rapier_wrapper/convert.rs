use rapier::prelude::*;
#[cfg(feature = "dim3")]
pub fn vector_to_rapier(vec: crate::Vector3) -> nalgebra::Vector3<Real> {
    nalgebra::Vector3::<Real>::new(vec.x, vec.y, vec.z)
}
#[cfg(feature = "dim2")]
pub fn vector_to_rapier(vec: crate::Vector2) -> nalgebra::Vector2<Real> {
    nalgebra::Vector2::<Real>::new(vec.x, vec.y)
}
#[cfg(feature = "dim3")]
pub fn vector_to_godot(vec: nalgebra::Vector3<Real>) -> crate::Vector {
    crate::Vector::new(vec.x, vec.y, vec.z)
}
#[cfg(feature = "dim2")]
pub fn vector_to_godot(vec: nalgebra::Vector2<Real>) -> godot::builtin::Vector2 {
    crate::Vector2::new(vec.x, vec.y)
}
#[cfg(feature = "dim3")]
pub fn angle_to_rapier(angle: Angle) -> AngVector<Real> {
    return vector_to_rapier(angle);
}
#[cfg(feature = "dim3")]
pub fn angle_to_godot(angle: AngVector<Real>) -> Angle {
    return vector_to_godot(angle);
}
#[cfg(feature = "dim2")]
pub fn angle_to_rapier(angle: Real) -> Real {
    angle
}
#[cfg(feature = "dim2")]
pub fn angle_to_godot(angle: Real) -> Real {
    angle
}
