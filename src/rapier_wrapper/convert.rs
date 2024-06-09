use std::ops::Mul;

use godot::builtin::Vector2;
use godot::builtin::Vector3;
use rapier::prelude::*;
use salva::math::AngularVector;

use crate::Angle;

const PIXELS_PER_METER: Real = 128.0;
const METERS_PER_PIXEL: Real = 1.0 / 128.0;

pub fn pixels_to_meters(x: Real) -> Real {
    PIXELS_PER_METER * x
}

pub fn vector_pixels_to_meters(v: Vector<Real>) -> Vector<Real> {
    v.mul(PIXELS_PER_METER)
}

pub fn meters_to_pixels(x: Real) -> Real {
    METERS_PER_PIXEL * x
}

pub fn vector_meters_to_pixels(v: Vector<Real>) -> Vector<Real> {
    v.mul(METERS_PER_PIXEL)
}

#[cfg(feature = "dim3")]
pub fn vector_to_rapier(vec: Vector3) -> nalgebra::Vector3<Real> {
    nalgebra::Vector3::<Real>::new(vec.x, vec.y, vec.z)
}

#[cfg(feature = "dim2")]
pub fn vector_to_rapier(vec: Vector2) -> nalgebra::Vector2<Real> {
    nalgebra::Vector2::<Real>::new(vec.x, vec.y)
}

#[cfg(feature = "dim3")]
pub fn vector_to_godot(vec: nalgebra::Vector3<Real>) -> Vector3 {
    Vector3::new(vec.x, vec.y, vec.z)
}

#[cfg(feature = "dim2")]
pub fn vector_to_godot(vec: nalgebra::Vector2<Real>) -> Vector2 {
    Vector2::new(vec.x, vec.y)
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
pub fn angle_to_rapier(angle: Angle) -> Real {
    return angle;
}

#[cfg(feature = "dim2")]
pub fn angle_to_godot(angle: Real) -> Angle {
    return angle;
}
