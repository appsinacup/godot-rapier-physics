use crate::rapier_wrapper::prelude::*;
use crate::Transform;
use rapier::prelude::*;

pub fn pixel_point_array_to_vec(pixel_data: Vec<Vector<Real>>) -> Vec<Point<Real>> {
    let mut vec = Vec::<Point<Real>>::with_capacity(pixel_data.len());
    for pixel_point in pixel_data {
        let point = vector_pixels_to_meters(pixel_point);
        vec.push(Point::<Real> { coords: point });
    }
    vec
}

#[derive(Copy, Clone, Debug)]
pub struct ShapeInfo {
    pub handle: Handle,
    pub pixel_position: Vector<Real>,
    pub rotation: AngVector<Real>,
    pub skew: Real,
    pub scale: Vector<Real>,
}

pub fn shape_info_from_body_shape(shape_handle: Handle, transform: Transform) -> ShapeInfo {
    let origin = transform.origin;
    let scale = transform.scale();
    ShapeInfo {
        handle: shape_handle,
        pixel_position: vector_to_rapier(origin),
        rotation: transform.rotation(),
        skew: transform.skew(),
        scale: vector_to_rapier(scale),
    }
}

pub fn shape_destroy(shape_handle: Handle) {
    physics_engine().remove_shape(shape_handle)
}
