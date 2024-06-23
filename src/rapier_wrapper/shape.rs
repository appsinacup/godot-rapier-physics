use rapier::prelude::*;
use types::Transform;

use super::ANG_ZERO;
use crate::rapier_wrapper::prelude::*;
use crate::*;
pub fn point_array_to_vec(pixel_data: &Vec<Vector<Real>>) -> Vec<Point<Real>> {
    let mut vec = Vec::<Point<Real>>::with_capacity(pixel_data.len());
    for point in pixel_data {
        vec.push(Point::<Real> { coords: *point });
    }
    vec
}
#[derive(Copy, Clone, Debug)]
pub struct ShapeInfo {
    pub handle: ShapeHandle,
    pub transform: Isometry<Real>,
    #[cfg(feature = "dim2")]
    pub skew: Real,
    pub scale: Vector<Real>,
}
#[cfg(feature = "dim2")]
pub fn shape_info_from_body_shape(shape_handle: ShapeHandle, transform: Transform) -> ShapeInfo {
    use nalgebra::Isometry2;
    ShapeInfo {
        handle: shape_handle,
        transform: Isometry2::new(vector_to_rapier(transform.origin), transform.rotation()),
        skew: transform.skew(),
        scale: vector_to_rapier(transform.scale()),
    }
}
#[cfg(feature = "dim3")]
pub fn shape_info_from_body_shape(shape_handle: ShapeHandle, transform: Transform) -> ShapeInfo {
    use nalgebra::Isometry3;
    let euler_angles = transform.basis.to_euler(godot::builtin::EulerOrder::XYZ);
    let isometry = Isometry3::new(
        vector_to_rapier(transform.origin),
        vector_to_rapier(euler_angles),
    );
    ShapeInfo {
        handle: shape_handle,
        transform: isometry,
        scale: vector_to_rapier(transform.basis.scale()),
    }
}
impl PhysicsEngine {
    #[cfg(feature = "dim2")]
    pub fn shape_create_convex_polyline(&mut self, points: &Vec<Vector<Real>>) -> ShapeHandle {
        let points_vec = point_array_to_vec(points);
        if let Some(shape_data) = SharedShape::convex_polyline(points_vec) {
            return self.insert_shape(shape_data);
        }
        ShapeHandle::default()
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_convex_polyline(&mut self, points: &Vec<Vector<Real>>) -> ShapeHandle {
        let points_vec = point_array_to_vec(points);
        if let Some(shape_data) = SharedShape::convex_hull(&points_vec) {
            return self.insert_shape(shape_data);
        }
        ShapeHandle::default()
    }

    #[cfg(feature = "dim2")]
    pub fn shape_create_box(&mut self, size: Vector<Real>) -> ShapeHandle {
        let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
        self.insert_shape(shape)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_box(&mut self, size: Vector<Real>) -> ShapeHandle {
        let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y, 0.5 * size.z);
        self.insert_shape(shape)
    }

    pub fn shape_create_halfspace(&mut self, normal: Vector<Real>, distance: Real) -> ShapeHandle {
        let shape = SharedShape::halfspace(UnitVector::new_normalize(normal));
        let shape_position = Isometry::new(normal * distance, ANG_ZERO);
        let mut shapes_vec = Vec::new();
        shapes_vec.push((shape_position, shape));
        let shape_compound = SharedShape::compound(shapes_vec);
        self.insert_shape(shape_compound)
    }

    pub fn shape_create_circle(&mut self, radius: Real) -> ShapeHandle {
        let shape = SharedShape::ball(radius);
        self.insert_shape(shape)
    }

    pub fn shape_create_capsule(&mut self, half_height: Real, radius: Real) -> ShapeHandle {
        let shape = SharedShape::capsule_y(half_height, radius);
        self.insert_shape(shape)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_cylinder(&mut self, half_height: Real, radius: Real) -> ShapeHandle {
        let shape = SharedShape::cylinder(half_height, radius);
        self.insert_shape(shape)
    }

    pub fn shape_create_concave_polyline(&mut self, points: &Vec<Vector<Real>>) -> ShapeHandle {
        let points_vec = point_array_to_vec(points);
        let shape = SharedShape::polyline(points_vec, None);
        self.insert_shape(shape)
    }

    pub fn shape_destroy(&mut self, shape_handle: ShapeHandle) {
        self.remove_shape(shape_handle)
    }
}
