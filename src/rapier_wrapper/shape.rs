use godot::builtin::Transform2D;
use nalgebra::Transform;
use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
pub fn point_array_to_vec(pixel_data: Vec<Vector<Real>>) -> Vec<Point<Real>> {
    let mut vec = Vec::<Point<Real>>::with_capacity(pixel_data.len());
    for point in pixel_data {
        vec.push(Point::<Real> { coords: point });
    }
    vec
}
#[derive(Copy, Clone, Debug)]
pub struct ShapeInfo {
    pub handle: Handle,
    pub transform: Isometry<Real>,
    pub skew: Real,
    pub scale: Vector<Real>,
}
pub fn shape_info_from_body_shape(shape_handle: Handle, pixel_transform: &Transform2D<Real>) -> ShapeInfo {
    let mut transform = pixel_transform.clone();
    transform.translation.vector = vector_pixels_to_meters(transform.translation.vector);
    ShapeInfo {
        handle: shape_handle,
        transform,
        skew,
        scale,
    }
}
pub fn shape_destroy(shape_handle: Handle) {
    physics_engine().remove_shape(shape_handle)
}
