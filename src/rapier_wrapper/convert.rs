use rapier::prelude::*;

use crate::Angle;
#[cfg(feature = "convert_pixels_to_meters")]
const PIXELS_PER_METER: Real = 128.0;
#[cfg(feature = "convert_pixels_to_meters")]
const METERS_PER_PIXEL: Real = 1.0 / 128.0;
#[cfg(not(feature = "convert_pixels_to_meters"))]
pub fn pixels_to_meters(x: Real) -> Real {
    x
}
#[cfg(not(feature = "convert_pixels_to_meters"))]
pub fn vector_pixels_to_meters(v: Vector<Real>) -> Vector<Real> {
    v
}
#[cfg(not(feature = "convert_pixels_to_meters"))]
pub fn angle_pixels_to_meters(v: AngVector<Real>) -> AngVector<Real> {
    v
}
#[cfg(not(feature = "convert_pixels_to_meters"))]
pub fn angle_meters_to_pixels(v: AngVector<Real>) -> AngVector<Real> {
    v
}
#[cfg(not(feature = "convert_pixels_to_meters"))]
pub fn meters_to_pixels(x: Real) -> Real {
    x
}
#[cfg(not(feature = "convert_pixels_to_meters"))]
pub fn tangent_meters_to_pixels(x: TangentImpulse<Real>) -> TangentImpulse<Real> {
    x
}
#[cfg(not(feature = "convert_pixels_to_meters"))]
pub fn vector_meters_to_pixels(v: Vector<Real>) -> Vector<Real> {
    v
}
#[cfg(feature = "convert_pixels_to_meters")]
pub fn pixels_to_meters(x: Real) -> Real {
    METERS_PER_PIXEL * x
}
#[cfg(feature = "convert_pixels_to_meters")]
pub fn vector_pixels_to_meters(v: Vector<Real>) -> Vector<Real> {
    v * METERS_PER_PIXEL
}
#[cfg(feature = "convert_pixels_to_meters")]
pub fn angle_pixels_to_meters(v: AngVector<Real>) -> AngVector<Real> {
    v * METERS_PER_PIXEL
}
#[cfg(feature = "convert_pixels_to_meters")]
pub fn angle_meters_to_pixels(v: AngVector<Real>) -> AngVector<Real> {
    v * PIXELS_PER_METER
}
#[cfg(feature = "convert_pixels_to_meters")]
pub fn meters_to_pixels(x: Real) -> Real {
    PIXELS_PER_METER * x
}
#[cfg(feature = "convert_pixels_to_meters")]
pub fn vector_meters_to_pixels(v: Vector<Real>) -> Vector<Real> {
    v * PIXELS_PER_METER
}
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
pub fn vector_to_godot(vec: nalgebra::Vector2<Real>) -> Vector2 {
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
pub fn angle_to_rapier(angle: Angle) -> Real {
    angle
}
#[cfg(feature = "dim2")]
pub fn angle_to_godot(angle: Real) -> Angle {
    angle
}
