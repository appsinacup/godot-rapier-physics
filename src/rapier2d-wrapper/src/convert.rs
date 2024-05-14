use rapier2d::prelude::*;
use crate::vector::Vector;

//const PIXELS_PER_METER : f64 = 1.0;
//const METERS_PER_PIXEL : f64 = 1.0 / 1.0;
const PIXELS_PER_METER : f64 = 128.0;
const METERS_PER_PIXEL : f64 = 1.0 / 128.0;

pub fn pixels_to_meters(x : Real) -> Real {
    if x == 0.0 { 0.0 } else { 
        let res = METERS_PER_PIXEL * (x as f64);
        return res as Real;
    }
}

pub fn vector_pixels_to_meters(v : &Vector) -> Vector {
    Vector{x: pixels_to_meters(v.x), y: pixels_to_meters(v.y)}
}

pub fn meters_to_pixels(x : Real) -> Real {
    let res = PIXELS_PER_METER * (x as f64);
    return res as Real;
}

pub fn vector_meters_to_pixels(v : &Vector) -> Vector {
    Vector{x: meters_to_pixels(v.x), y: meters_to_pixels(v.y)}
}
