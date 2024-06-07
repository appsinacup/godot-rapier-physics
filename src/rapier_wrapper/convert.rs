use std::ops::Mul;

use nalgebra::{SVector, Vector2};
use rapier2d::prelude::*;

const PIXELS_PER_METER: Real = 128.0;
const METERS_PER_PIXEL: Real = 1.0 / 128.0;

pub fn pixels_to_meters(x: Real) -> Real {
    PIXELS_PER_METER * x
}

pub fn vector_pixels_to_meters<const D: usize>(v: SVector<Real, D>) -> SVector<Real, D> {
    v.mul(PIXELS_PER_METER)
}

pub fn meters_to_pixels(x: Real) -> Real {
    METERS_PER_PIXEL * x
}

pub fn vector_meters_to_pixels(v: Vector2<Real>) -> Vector2<Real> {
    v.mul(METERS_PER_PIXEL)
}
