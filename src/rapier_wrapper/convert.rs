#[cfg(feature = "dim3")]
use rapier::prelude::AngVector;
#[cfg(feature = "dim2")]
use rapier::prelude::Real;
use rapier::prelude::Vector;

use crate::types::Angle;
#[cfg(feature = "dim3")]
pub fn vector_to_rapier(vec: crate::Vector3) -> Vector {
    Vector::new(vec.x, vec.y, vec.z)
}
#[cfg(feature = "dim2")]
pub fn vector_to_rapier(vec: crate::Vector2) -> Vector {
    Vector::new(vec.x, vec.y)
}
#[cfg(feature = "dim3")]
pub fn vector_to_godot<T: Into<Vector>>(vec: T) -> godot::builtin::Vector3 {
    let vec: Vector = vec.into();
    crate::Vector3::new(vec.x, vec.y, vec.z)
}
#[cfg(feature = "dim2")]
pub fn vector_to_godot<T: Into<Vector>>(vec: T) -> godot::builtin::Vector2 {
    let vec: Vector = vec.into();
    crate::Vector2::new(vec.x, vec.y)
}
#[cfg(feature = "dim3")]
pub fn angle_to_rapier(angle: Angle) -> AngVector {
    vector_to_rapier(angle)
}
#[cfg(feature = "dim3")]
pub fn angle_to_godot(angle: AngVector) -> Angle {
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
        Vector::new(aabb.position.x, aabb.position.y),
        Vector::new(aabb.position.x + aabb.size.x, aabb.position.y + aabb.size.y),
    )
}
#[cfg(feature = "dim3")]
pub fn aabb_to_salva_aabb(aabb: godot::prelude::Aabb) -> salva::parry::bounding_volume::Aabb {
    salva::parry::bounding_volume::Aabb::new(
        Vector::new(aabb.position.x, aabb.position.y, aabb.position.z),
        Vector::new(
            aabb.position.x + aabb.size.x,
            aabb.position.y + aabb.size.y,
            aabb.position.z + aabb.size.z,
        ),
    )
}
