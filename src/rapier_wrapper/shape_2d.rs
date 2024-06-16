use rapier::prelude::*;

use super::shape::point_array_to_vec;
use crate::rapier_wrapper::prelude::*;
pub fn shape_create_box(size: Vector<Real>) -> Handle {
    let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
    physics_engine().insert_shape(shape)
}
pub fn shape_create_convex_polyline(points: Vec<Vector<Real>>) -> Handle {
    let points_vec = point_array_to_vec(points);
    if let Some(shape_data) = SharedShape::convex_polyline(points_vec) {
        return physics_engine().insert_shape(shape_data);
    }
    Handle::default()
}
