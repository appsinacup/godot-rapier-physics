use rapier::prelude::*;

use crate::types::*;
use crate::*;
#[cfg(feature = "dim3")]
pub fn vector_to_rapier(vec: crate::Vector3) -> nalgebra::Vector3<Real> {
    nalgebra::Vector3::<Real>::new(vec.x, vec.y, vec.z)
}
#[cfg(feature = "dim2")]
pub fn vector_to_rapier(vec: crate::Vector2) -> nalgebra::Vector2<Real> {
    nalgebra::Vector2::<Real>::new(vec.x, vec.y)
}
#[cfg(feature = "dim3")]
pub fn vector_to_godot(vec: nalgebra::Vector3<Real>) -> godot::builtin::Vector3 {
    crate::Vector3::new(vec.x, vec.y, vec.z)
}
#[cfg(feature = "dim2")]
pub fn vector_to_godot(vec: nalgebra::Vector2<Real>) -> godot::builtin::Vector2 {
    crate::Vector2::new(vec.x, vec.y)
}
#[cfg(feature = "dim3")]
pub fn angle_to_rapier(angle: Angle) -> AngVector<Real> {
    vector_to_rapier(angle)
}
#[cfg(feature = "dim3")]
pub fn angle_to_godot(angle: AngVector<Real>) -> Angle {
    vector_to_godot(angle)
}
#[cfg(feature = "dim2")]
pub fn angle_to_rapier(angle: Angle) -> Real {
    angle
}
#[cfg(feature = "dim2")]
pub fn angle_to_godot(angle: Real) -> Angle {
    angle
}
#[cfg(feature = "dim2")]
pub fn aabb_to_salva_aabb(aabb: godot::prelude::Rect2) -> salva::parry::bounding_volume::Aabb {
    salva::parry::bounding_volume::Aabb::new(
        nalgebra::Point2::new(aabb.position.x, aabb.position.y),
        nalgebra::Point2::new(aabb.position.x + aabb.size.x, aabb.position.y + aabb.size.y),
    )
}
#[cfg(feature = "dim3")]
pub fn aabb_to_salva_aabb(aabb: godot::prelude::Aabb) -> salva::parry::bounding_volume::Aabb {
    salva::parry::bounding_volume::Aabb::new(
        nalgebra::Point3::new(aabb.position.x, aabb.position.y, aabb.position.z),
        nalgebra::Point3::new(
            aabb.position.x + aabb.size.x,
            aabb.position.y + aabb.size.y,
            aabb.position.z + aabb.size.z,
        ),
    )
}
