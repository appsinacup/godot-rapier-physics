use rapier::prelude::*;

use super::shape::point_array_to_vec;
use crate::rapier_wrapper::prelude::*;
pub fn shape_create_box(size: Vector<Real>) -> Handle {
    let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
    physics_engine().insert_shape(shape)
}
pub fn shape_create_halfspace(normal: Vector<Real>, distance: Real) -> Handle {
    let shape = SharedShape::halfspace(UnitVector::new_normalize(Vector::new(normal.x, -normal.y)));
    let shape_position = Isometry::new(normal * distance, 0.0);
    let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::new();
    shapes_vec.push((shape_position, shape));
    let shape_compound = SharedShape::compound(shapes_vec);
    physics_engine().insert_shape(shape_compound)
}
pub fn shape_create_circle(radius: Real) -> Handle {
    let shape = SharedShape::ball(radius);
    physics_engine().insert_shape(shape)
}
pub fn shape_create_capsule(half_height: Real, radius: Real) -> Handle {
    let shape = SharedShape::capsule_y(half_height, radius);
    physics_engine().insert_shape(shape)
}
pub fn shape_create_convex_polyline(points: Vec<Vector<Real>>) -> Handle {
    let points_vec = point_array_to_vec(points);
    if let Some(shape_data) = SharedShape::convex_polyline(points_vec) {
        return physics_engine().insert_shape(shape_data);
    }
    Handle::default()
}
pub fn shape_create_concave_polyline(points: Vec<Vector<Real>>) -> Handle {
    let points_vec = point_array_to_vec(points);
    let shape = SharedShape::polyline(points_vec, None);
    physics_engine().insert_shape(shape)
}
