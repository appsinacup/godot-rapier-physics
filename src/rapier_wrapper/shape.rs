use nalgebra::Isometry2;
use nalgebra::Isometry3;
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
    pub transform: Isometry<Real>,
    #[cfg(feature = "dim2")]
    pub skew: Real,
    pub scale: Vector<Real>,
}
#[cfg(feature = "dim2")]
pub fn shape_info_from_body_shape(shape_handle: Handle, transform: Transform) -> ShapeInfo {
    ShapeInfo {
        handle: shape_handle,
        transform: Isometry2::new(vector_to_rapier(transform.origin), transform.rotation()),
        skew: transform.skew(),
        scale: vector_to_rapier(transform.scale()),
    }
}
#[cfg(feature = "dim3")]
pub fn shape_info_from_body_shape(shape_handle: Handle, transform: Transform) -> ShapeInfo {
    let euler_angles = transform.basis.to_euler(godot::builtin::EulerOrder::XYZ);
    let isometry = Isometry3::new(vector_to_rapier(transform.origin), vector_to_rapier(euler_angles));
    ShapeInfo {
        handle: shape_handle,
        transform: isometry,
        scale: vector_to_rapier(transform.basis.scale()),
    }
}
pub fn shape_destroy(shape_handle: Handle) {
    physics_engine().remove_shape(shape_handle)
}
