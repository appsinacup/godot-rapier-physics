use nalgebra::Isometry2;
use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::Transform;
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
    pub transform: Isometry2<Real>,
    pub skew: Real,
    pub scale: Vector<Real>,
}
pub fn shape_info_from_body_shape(shape_handle: Handle, transform: Transform) -> ShapeInfo {
    ShapeInfo {
        handle: shape_handle,
        transform: Isometry2::new(vector_to_rapier(transform.origin), transform.rotation()),
        skew: transform.skew(),
        scale: vector_to_rapier(transform.scale()),
    }
}
pub fn shape_destroy(shape_handle: Handle) {
    physics_engine().remove_shape(shape_handle)
}
