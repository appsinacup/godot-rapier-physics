use rapier2d::prelude::*;
use crate::vector::Vector;

pub fn pixels_to_meters(x : Real) -> Real {
    x / 100.0
}

pub fn vector_pixels_to_meters(v : &Vector) -> Vector {
    Vector{x: pixels_to_meters(v.x), y: pixels_to_meters(v.y)}
}

pub fn meters_to_pixels(x : Real) -> Real {
    x * 100.0
}

pub fn vector_meters_to_pixels(v : &Vector) -> Vector {
    Vector{x: meters_to_pixels(v.x), y: meters_to_pixels(v.y)}
}