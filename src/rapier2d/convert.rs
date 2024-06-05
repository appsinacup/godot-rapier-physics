use nalgebra::Vector2;
use rapier2d::prelude::*;

const PIXELS_PER_METER: f64 = 128.0;
const METERS_PER_PIXEL: f64 = 1.0 / 128.0;

pub fn pixels_to_meters(x: Real) -> Real {
    if x == 0.0 {
        0.0
    } else {
        let res = METERS_PER_PIXEL * (x as f64);
        res as Real
    }
}

pub fn vector_pixels_to_meters(v: Vector2<Real>) -> Vector2<Real> {
    Vector2::new(pixels_to_meters(v.x), pixels_to_meters(v.y))
}

pub fn meters_to_pixels(x: Real) -> Real {
    let res = PIXELS_PER_METER * (x as f64);
    res as Real
}

pub fn vector_meters_to_pixels(v: Vector2<Real>) -> Vector2<Real> {
    Vector2::new(meters_to_pixels(v.x), meters_to_pixels(v.y))
}
