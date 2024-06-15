use rapier::prelude::*;
use super::transform::Transform;

#[cfg(feature = "dim3")]
pub fn vector_to_rapier(vec: crate::Vector3) -> nalgebra::Vector3<Real> {
    nalgebra::Vector3::<Real>::new(vec.x, vec.y, vec.z)
}
#[cfg(feature = "dim2")]
pub fn vector_to_rapier(vec: crate::Vector2) -> nalgebra::Vector2<Real> {
    nalgebra::Vector2::<Real>::new(vec.x, vec.y)
}
#[cfg(feature = "dim2")]
pub fn transform_to_rapier(transform: crate::Transform2D) -> Transform {
    let a = transform.a;
    let b = transform.b;
    let origin = vector_to_rapier(transform.origin);
    let isometry = Isometry::from_parts(translation, rotation);
    transform.scale()
    isometry.rotation = Rotation::from_matrix(m)
    let rotation = isometry.rotation;
    rotation.
    return Transform {
        isometry: Isometry::new(origin, transform.rotation().angle()),
        scale: vector_to_rapier(transform.scale()),
    }
}
#[cfg(feature = "dim3")]
pub fn transform_to_rapier(transform: crate::Transform3D) -> Transform {
    transform.basis.rows
    transform.basis.col_a()
    let a = transform.a;
    let b = transform.b;
    let origin = vector_to_rapier(transform.origin);
    let isometry = Isometry::from_parts(translation, rotation);
    isometry.rotation = Rotation::from_matrix(m)
    let rotation = isometry.rotation;
    rotation.
    return Transform {
        isometry: Isometry::new(origin, transform.rotation().angle()),
        scale: vector_to_rapier(transform.scale()),
    }
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

#[cfg(feature = "dim2")]
pub fn angle_cross(vec1: Vector<Real>, vec2: Vector<Real>) -> Real {
    return vec1.perp(&vec2);
}

#[cfg(feature = "dim3")]
pub fn angle_cross(vec1: Vector<Real>, vec2: Vector<Real>) -> Vector<Real> {
    return vec1.cross(&vec2);
}