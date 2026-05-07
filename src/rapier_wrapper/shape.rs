use rapier::prelude::*;
use types::Transform;

use crate::rapier_wrapper::prelude::*;
use crate::*;
pub fn point_array_to_vec(pixel_data: &Vec<Vector>) -> Vec<Vector> {
    let mut vec = Vec::<Vector>::with_capacity(pixel_data.len());
    for point in pixel_data {
        vec.push(*point);
    }
    vec
}
pub fn vec_to_point_array(pixel_data: &[Vector]) -> Vec<Vector> {
    let mut vec = Vec::<Vector>::with_capacity(pixel_data.len());
    for point in pixel_data {
        vec.push(*point);
    }
    vec
}
#[cfg(feature = "dim2")]
fn signed_area(points: &[Vector]) -> Real {
    let mut area = 0.0;
    for i in 0..points.len() {
        let next = (i + 1) % points.len();
        area += points[i].x * points[next].y - points[next].x * points[i].y;
    }
    area * 0.5
}
#[cfg(feature = "dim2")]
enum ConvexPolylineOrientation {
    CounterClockwise,
    Clockwise,
}
#[cfg(feature = "dim2")]
fn convex_polyline_orientation(points: &[Vector]) -> Option<ConvexPolylineOrientation> {
    if points.len() < 3 {
        return None;
    }
    let epsilon = 0.000001;
    let area = signed_area(points);
    let orientation = if area > epsilon {
        ConvexPolylineOrientation::CounterClockwise
    } else if area < -epsilon {
        ConvexPolylineOrientation::Clockwise
    } else {
        return None;
    };
    for i in 0..points.len() {
        let current = points[i];
        let next = points[(i + 1) % points.len()];
        if (next - current).length_squared() <= epsilon * epsilon {
            return None;
        }
        let after_next = points[(i + 2) % points.len()];
        let edge_a = next - current;
        let edge_b = after_next - next;
        let cross = edge_a.x * edge_b.y - edge_a.y * edge_b.x;
        match orientation {
            ConvexPolylineOrientation::CounterClockwise => {
                if cross < -epsilon {
                    return None;
                }
            }
            ConvexPolylineOrientation::Clockwise => {
                if cross > epsilon {
                    return None;
                }
            }
        }
    }
    Some(orientation)
}
#[cfg(feature = "dim2")]
fn sort_points_counter_clockwise(points: &[Vector]) -> Vec<Vector> {
    let mut center = Vector::ZERO;
    for point in points {
        center += *point;
    }
    center /= points.len() as Real;
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| {
        let angle_a = (a.y - center.y).atan2(a.x - center.x);
        let angle_b = (b.y - center.y).atan2(b.x - center.x);
        angle_a
            .partial_cmp(&angle_b)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                let distance_a = (*a - center).length_squared();
                let distance_b = (*b - center).length_squared();
                distance_a
                    .partial_cmp(&distance_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    });
    sorted
}
#[cfg(feature = "dim2")]
fn create_convex_polyline_unmodified(points: Vec<Vector>) -> Option<SharedShape> {
    if matches!(
        convex_polyline_orientation(&points),
        Some(ConvexPolylineOrientation::CounterClockwise)
    ) {
        return SharedShape::convex_polyline_unmodified(points);
    }
    None
}
#[derive(Copy, Clone, Debug)]
pub struct ShapeInfo {
    pub handle: ShapeHandle,
    pub transform: Pose,
    #[cfg(feature = "dim2")]
    pub skew: Real,
    pub scale: Vector,
}
#[cfg(feature = "dim2")]
pub fn shape_info_from_body_shape(shape_handle: ShapeHandle, transform: Transform) -> ShapeInfo {
    ShapeInfo {
        handle: shape_handle,
        transform: Pose::from_parts(
            vector_to_rapier(transform.origin),
            Rotation::from_angle(transform.rotation()),
        ),
        skew: transform.skew(),
        scale: vector_to_rapier(transform.scale()),
    }
}
#[cfg(feature = "dim3")]
pub fn shape_info_from_body_shape(shape_handle: ShapeHandle, transform: Transform) -> ShapeInfo {
    let quaternion = transform.basis.get_quaternion();
    let rotation = Rotation::from_xyzw(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    ShapeInfo {
        handle: shape_handle,
        transform: Pose::from_parts(vector_to_rapier(transform.origin), rotation),
        scale: vector_to_rapier(transform.basis.get_scale()),
    }
}
impl PhysicsEngine {
    #[cfg(feature = "dim2")]
    pub fn shape_create_convex_polyline(
        &mut self,
        points: &Vec<Vector>,
        handle: ShapeHandle,
    ) -> bool {
        let points_vec = point_array_to_vec(points);
        let mut polyline_points = points_vec.clone();
        match convex_polyline_orientation(&polyline_points) {
            Some(ConvexPolylineOrientation::CounterClockwise) => {}
            Some(ConvexPolylineOrientation::Clockwise) => polyline_points.reverse(),
            None => polyline_points = sort_points_counter_clockwise(&points_vec),
        }
        if let Some(shape_data) = create_convex_polyline_unmodified(polyline_points) {
            self.insert_shape(shape_data, handle);
            return true;
        }
        if let Some(shape_data) = SharedShape::convex_hull(&points_vec) {
            self.insert_shape(shape_data, handle);
            return true;
        }
        false
    }

    #[cfg(feature = "dim2")]
    pub fn shape_get_convex_polyline_points(&self, handle: ShapeHandle) -> Vec<Vector> {
        if let Some(shape) = self.get_shape(handle)
            && let Some(shape) = shape.as_convex_polygon()
        {
            let points = shape.points();
            let points_vec = vec_to_point_array(points);
            return points_vec;
        }
        vec![]
    }

    #[cfg(feature = "dim3")]
    pub fn shape_get_convex_polyline_points(&self, handle: ShapeHandle) -> Vec<Vector> {
        if let Some(shape) = self.get_shape(handle)
            && let Some(shape) = shape.as_convex_polyhedron()
        {
            let points = shape.points();
            let points_vec = vec_to_point_array(points);
            return points_vec;
        }
        vec![]
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_convex_polyline(
        &mut self,
        points: &Vec<Vector>,
        handle: ShapeHandle,
    ) -> bool {
        let points_vec = point_array_to_vec(points);
        if let Some(shape_data) = SharedShape::convex_hull(&points_vec) {
            self.insert_shape(shape_data, handle);
            return true;
        }
        false
    }

    #[cfg(feature = "dim2")]
    pub fn shape_create_box(&mut self, size: Vector, handle: ShapeHandle) {
        let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
        self.insert_shape(shape, handle);
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_box(&mut self, size: Vector, handle: ShapeHandle) {
        let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y, 0.5 * size.z);
        self.insert_shape(shape, handle);
    }

    pub fn shape_get_box_size(&self, shape_handle: ShapeHandle) -> Vector {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_cuboid()
        {
            return shape.half_extents;
        }
        Vector::ZERO
    }

    pub fn shape_create_halfspace(&mut self, normal: Vector, distance: Real, handle: ShapeHandle) {
        let shape = SharedShape::halfspace(normal.normalize_or_zero());
        let shape_position = Pose::from_parts(normal * distance, Rotation::default());
        let shapes_vec = vec![(shape_position, shape)];
        let shape_compound = SharedShape::compound(shapes_vec);
        self.insert_shape(shape_compound, handle);
    }

    pub fn shape_get_halfspace(&self, shape_handle: ShapeHandle) -> (Vector, Real) {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_compound()
            && let Some(shape) = shape.shapes().first()
        {
            let normal = shape.0.translation;
            return (normal.normalize_or_zero(), normal.length());
        }
        (Vector::ZERO, 0.0)
    }

    pub fn shape_create_circle(&mut self, radius: Real, handle: ShapeHandle) {
        let shape = SharedShape::ball(radius);
        self.insert_shape(shape, handle);
    }

    pub fn shape_circle_get_radius(&self, shape_handle: ShapeHandle) -> Real {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_ball()
        {
            return shape.radius;
        }
        0.0
    }

    pub fn shape_create_capsule(&mut self, half_height: Real, radius: Real, handle: ShapeHandle) {
        let shape = SharedShape::capsule_y(half_height, radius);
        self.insert_shape(shape, handle);
    }

    pub fn shape_get_capsule(&self, shape_handle: ShapeHandle) -> (Real, Real) {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_capsule()
        {
            return (shape.half_height(), shape.radius);
        }
        (0.0, 0.0)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_cylinder(&mut self, half_height: Real, radius: Real, handle: ShapeHandle) {
        let shape = SharedShape::cylinder(half_height, radius);
        self.insert_shape(shape, handle)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_get_cylinder(&self, shape_handle: ShapeHandle) -> (Real, Real) {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_cylinder()
        {
            return (shape.half_height, shape.radius);
        }
        (0.0, 0.0)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_heightmap(
        &mut self,
        heights: &[Real],
        width: i32,
        depth: i32,
        handle: ShapeHandle,
    ) {
        use rapier::parry::utils::Array2;
        let width = width as usize;
        let depth = depth as usize;
        let mut rotated_data = Vec::with_capacity(width * depth);
        for j in 0..width {
            for i in 0..depth {
                rotated_data.push(heights[i * width + j]);
            }
        }
        let rotated_heights = Array2::new(depth, width, rotated_data);
        // The new dimensions after rotating the heightmap
        let new_width = depth;
        let new_depth = width;
        // Create the shape with the rotated dimensions
        let shape = SharedShape::heightfield_with_flags(
            rotated_heights,
            Vector::new(new_depth as Real, 1.0, new_width as Real),
            HeightFieldFlags::FIX_INTERNAL_EDGES,
        );
        self.insert_shape(shape, handle)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_get_heightmap(&self, shape_handle: ShapeHandle) -> (Vec<Real>, i32, i32) {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_heightfield()
        {
            let scale = shape.scale();
            let depth = scale.x as i32;
            let width = scale.z as i32;
            return (shape.heights().data().to_vec(), depth, width);
        }
        (Vec::new(), 0, 0)
    }

    #[cfg(feature = "dim2")]
    pub fn shape_create_concave_polyline(
        &mut self,
        points: &Vec<Vector>,
        indices: Option<Vec<[u32; 2]>>,
        handle: ShapeHandle,
    ) {
        if points.is_empty() {
            self.remove_shape(handle);
            return;
        }
        let points_vec = point_array_to_vec(points);
        let shape = SharedShape::polyline(points_vec, indices);
        self.insert_shape(shape, handle);
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_concave_polyline(
        &mut self,
        points: &Vec<Vector>,
        indices: Option<Vec<[u32; 3]>>,
        handle: ShapeHandle,
    ) {
        if points.is_empty() {
            self.remove_shape(handle);
            return;
        }
        let points_vec = point_array_to_vec(points);
        let Some(indices) = indices else {
            godot_error!("Trimesh requires indices");
            self.remove_shape(handle);
            return;
        };
        if indices.is_empty() {
            godot_error!("Trimesh cannot have empty indices");
            self.remove_shape(handle);
            return;
        }
        let shape =
            SharedShape::trimesh_with_flags(points_vec, indices, TriMeshFlags::FIX_INTERNAL_EDGES);
        match shape {
            Ok(s) => self.insert_shape(s, handle),
            Err(e) => {
                godot_error!("Failed to create trimesh: {:?}", e);
                self.remove_shape(handle);
            }
        }
    }

    #[cfg(feature = "dim2")]
    pub fn shape_get_concave_polyline(
        &self,
        shape_handle: ShapeHandle,
    ) -> (&[Vector], &[[u32; 2]]) {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_polyline()
        {
            return (shape.vertices(), shape.indices());
        }
        (&[], &[])
    }

    #[cfg(feature = "dim3")]
    pub fn shape_get_concave_polyline(
        &self,
        shape_handle: ShapeHandle,
    ) -> (&[Vector], &[[u32; 3]]) {
        if let Some(shape) = self.get_shape(shape_handle)
            && let Some(shape) = shape.as_trimesh()
        {
            return (shape.vertices(), shape.indices());
        }
        (&[], &[])
    }

    pub fn shape_get_aabb(&self, handle: ShapeHandle) -> rapier::prelude::Aabb {
        if let Some(shape) = self.get_shape(handle) {
            return shape.compute_local_aabb();
        }
        rapier::prelude::Aabb::new_invalid()
    }

    pub fn shape_destroy(&mut self, shape_handle: ShapeHandle) {
        self.remove_shape(shape_handle)
    }
}
#[cfg(all(test, feature = "dim2"))]
mod tests {
    use super::*;
    #[test]
    fn convex_shape_preserves_ccw_convex_polyline_points() {
        let mut physics_engine = PhysicsEngine::default();
        let handle = 1;
        let points = vec![
            Vector::new(0.0, 0.0),
            Vector::new(1.0, 0.0),
            Vector::new(2.0, 0.0),
            Vector::new(2.0, 1.0),
            Vector::new(0.0, 1.0),
        ];
        assert!(physics_engine.shape_create_convex_polyline(&points, handle));
        let shape_points = physics_engine.shape_get_convex_polyline_points(handle);
        assert_eq!(shape_points, points);
    }
    #[test]
    fn convex_shape_preserves_clockwise_convex_polyline_points() {
        let mut physics_engine = PhysicsEngine::default();
        let handle = 1;
        let points = vec![
            Vector::new(0.0, 1.0),
            Vector::new(2.0, 1.0),
            Vector::new(2.0, 0.0),
            Vector::new(1.0, 0.0),
            Vector::new(0.0, 0.0),
        ];
        assert!(physics_engine.shape_create_convex_polyline(&points, handle));
        let shape_points = physics_engine.shape_get_convex_polyline_points(handle);
        assert_eq!(shape_points.len(), points.len());
        assert!(signed_area(&shape_points) > 0.0);
        for point in points {
            assert!(shape_points.contains(&point));
        }
    }
    #[test]
    fn convex_shape_accepts_unordered_point_cloud() {
        let mut physics_engine = PhysicsEngine::default();
        let handle = 1;
        let points = vec![
            Vector::new(0.0, 0.0),
            Vector::new(1.0, 1.0),
            Vector::new(1.0, 0.0),
            Vector::new(0.0, 1.0),
        ];
        assert!(physics_engine.shape_create_convex_polyline(&points, handle));
        let hull_points = physics_engine.shape_get_convex_polyline_points(handle);
        assert_eq!(hull_points.len(), 4);
        assert!(signed_area(&hull_points) > 0.0);
        for point in points {
            assert!(hull_points.contains(&point));
        }
    }
}
