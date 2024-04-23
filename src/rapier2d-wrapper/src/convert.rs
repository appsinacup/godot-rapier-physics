use rapier2d::prelude::*;
use crate::vector::Vector;

const PIXELS_PER_METER : Real = 1.0;

pub fn pixels_to_meters(x : Real) -> Real {
    return x;
    if x == 0.0 { 0.0 } else { x / PIXELS_PER_METER }
}

pub fn vector_pixels_to_meters(v : &Vector) -> Vector {
    Vector{x: pixels_to_meters(v.x), y: pixels_to_meters(v.y)}
}

pub fn meters_to_pixels(x : Real) -> Real {
    return x;
    x * PIXELS_PER_METER
}

pub fn vector_meters_to_pixels(v : &Vector) -> Vector {
    Vector{x: meters_to_pixels(v.x), y: meters_to_pixels(v.y)}
}
