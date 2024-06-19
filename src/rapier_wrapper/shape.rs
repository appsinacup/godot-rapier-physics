use rapier::prelude::*;

use super::ANG_ZERO;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::PhysicsData;
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
    use nalgebra::Isometry2;
    ShapeInfo {
        handle: shape_handle,
        transform: Isometry2::new(vector_to_rapier(transform.origin), transform.rotation()),
        skew: transform.skew(),
        scale: vector_to_rapier(transform.scale()),
    }
}
#[cfg(feature = "dim3")]
pub fn shape_info_from_body_shape(shape_handle: Handle, transform: Transform) -> ShapeInfo {
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
#[cfg(feature = "dim2")]
pub fn shape_create_convex_polyline(points: Vec<Vector<Real>>, physics_engine: &mut PhysicsEngine) -> Handle {
    let points_vec = point_array_to_vec(points);
    if let Some(shape_data) = SharedShape::convex_polyline(points_vec) {
        return physics_engine.insert_shape(shape_data);
    }
    Handle::default()
}
#[cfg(feature = "dim3")]
pub fn shape_create_convex_polyline(points: Vec<Vector<Real>>, physics_engine: &mut PhysicsEngine) -> Handle {
    let points_vec = point_array_to_vec(points);
    if let Some(shape_data) = SharedShape::convex_hull(&points_vec) {
        return physics_engine.insert_shape(shape_data);
    }
    Handle::default()
}
#[cfg(feature = "dim2")]
pub fn shape_create_box(size: Vector<Real>, physics_engine: &mut PhysicsEngine) -> Handle {
    let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
    physics_engine.insert_shape(shape)
}
#[cfg(feature = "dim3")]
pub fn shape_create_box(size: Vector<Real>, physics_engine: &mut PhysicsEngine) -> Handle {
    let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y, 0.5 * size.z);
    physics_engine.insert_shape(shape)
}
pub fn shape_create_halfspace(normal: Vector<Real>, distance: Real, physics_engine: &mut PhysicsEngine) -> Handle {
    let shape = SharedShape::halfspace(UnitVector::new_normalize(normal));
    let shape_position = Isometry::new(normal * distance, ANG_ZERO);
    let mut shapes_vec = Vec::new();
    shapes_vec.push((shape_position, shape));
    let shape_compound = SharedShape::compound(shapes_vec);
    physics_engine.insert_shape(shape_compound)
}
pub fn shape_create_circle(radius: Real, physics_engine: &mut PhysicsEngine) -> Handle {
    let shape = SharedShape::ball(radius);
    physics_engine.insert_shape(shape)
}
pub fn shape_create_capsule(half_height: Real, radius: Real, physics_engine: &mut PhysicsEngine) -> Handle {
    let shape = SharedShape::capsule_y(half_height, radius);
    physics_engine.insert_shape(shape)
}
#[cfg(feature = "dim3")]
pub fn shape_create_cylinder(half_height: Real, radius: Real, physics_engine: &mut PhysicsEngine) -> Handle {
    let shape = SharedShape::cylinder(half_height, radius);
    physics_engine.insert_shape(shape)
}
pub fn shape_create_concave_polyline(points: Vec<Vector<Real>>, physics_engine: &mut PhysicsEngine) -> Handle {
    let points_vec = point_array_to_vec(points);
    let shape = SharedShape::polyline(points_vec, None);
    physics_engine.insert_shape(shape)
}
pub fn shape_destroy(shape_handle: Handle, physics_engine: &mut PhysicsEngine) {
    physics_engine.remove_shape(shape_handle)
}
