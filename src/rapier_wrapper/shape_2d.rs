use rapier::prelude::*;

use super::shape::pixel_point_array_to_vec;
use crate::rapier_wrapper::prelude::*;
pub fn shape_create_box(pixel_size: Vector<Real>) -> Handle {
    let size = vector_pixels_to_meters(pixel_size);
    let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
    physics_engine().insert_shape(shape)
}
pub fn shape_create_halfspace(normal: Vector<Real>, pixel_distance: Real) -> Handle {
    let distance = pixels_to_meters(pixel_distance);
    let shape = SharedShape::halfspace(UnitVector::new_normalize(Vector::new(normal.x, -normal.y)));
    let shape_position = Isometry::new(normal * distance, 0.0);
    let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::new();
    shapes_vec.push((shape_position, shape));
    let shape_compound = SharedShape::compound(shapes_vec);
    physics_engine().insert_shape(shape_compound)
}
pub fn shape_create_circle(pixel_radius: Real) -> Handle {
    let radius = pixels_to_meters(pixel_radius);
    let shape = SharedShape::ball(radius);
    physics_engine().insert_shape(shape)
}
pub fn shape_create_capsule(pixel_half_height: Real, pixel_radius: Real) -> Handle {
    let half_height = pixels_to_meters(pixel_half_height);
    let radius = pixels_to_meters(pixel_radius);
    let shape = SharedShape::capsule_y(half_height, radius);
    physics_engine().insert_shape(shape)
}
pub fn shape_create_convex_polyline(pixel_points: Vec<Vector<Real>>) -> Handle {
    let points_vec = pixel_point_array_to_vec(pixel_points);
    if let Some(shape_data) = SharedShape::convex_polyline(points_vec) {
        return physics_engine().insert_shape(shape_data);
    }
    Handle::default()
}
pub fn shape_create_concave_polyline(pixel_points: Vec<Vector<Real>>) -> Handle {
    let points_vec = pixel_point_array_to_vec(pixel_points);
    let shape = SharedShape::polyline(points_vec, None);
    physics_engine().insert_shape(shape)
}
