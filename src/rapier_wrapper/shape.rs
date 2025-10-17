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
pub fn vec_to_point_array(pixel_data: &[Point<Real>]) -> Vec<Vector<Real>> {
    let mut vec = Vec::<Vector<Real>>::with_capacity(pixel_data.len());
    for point in pixel_data {
        vec.push(Vector::<Real>::from(point.coords));
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
    use nalgebra::Quaternion;
    use nalgebra::Translation3;
    let quaternion = transform.basis.get_quaternion();
    let rotation = Rotation::from_quaternion(Quaternion::new(
        quaternion.w,
        quaternion.x,
        quaternion.y,
        quaternion.z,
    ));
    let translation = Translation3::from(vector_to_rapier(transform.origin));
    let isometry = Isometry3::from_parts(translation, rotation);
    ShapeInfo {
        handle: shape_handle,
        transform: isometry,
        scale: vector_to_rapier(transform.basis.get_scale()),
    }
}
impl PhysicsEngine {
    #[cfg(feature = "dim2")]
    pub fn shape_create_convex_polyline(
        &mut self,
        points: &Vec<Vector<Real>>,
        handle: ShapeHandle,
    ) {
        let points_vec = point_array_to_vec(points);
        if let Some(shape_data) = SharedShape::convex_polyline_unmodified(points_vec) {
            self.insert_shape(shape_data, handle)
        }
    }

    #[cfg(feature = "dim2")]
    pub fn shape_get_convex_polyline_points(&self, handle: ShapeHandle) -> Vec<Vector<Real>> {
        if let Some(shape) = self.get_shape(handle) {
            if let Some(shape) = shape.as_convex_polygon() {
                let points = shape.points();
                let points_vec = vec_to_point_array(points);
                return points_vec;
            }
        }
        vec![]
    }

    #[cfg(feature = "dim3")]
    pub fn shape_get_convex_polyline_points(&self, handle: ShapeHandle) -> Vec<Vector<Real>> {
        if let Some(shape) = self.get_shape(handle) {
            if let Some(shape) = shape.as_convex_polyhedron() {
                let points = shape.points();
                let points_vec = vec_to_point_array(points);
                return points_vec;
            }
        }
        vec![]
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_convex_polyline(
        &mut self,
        points: &Vec<Vector<Real>>,
        handle: ShapeHandle,
    ) {
        let points_vec = point_array_to_vec(points);
        if let Some(shape_data) = SharedShape::convex_hull(&points_vec) {
            self.insert_shape(shape_data, handle)
        }
    }

    #[cfg(feature = "dim2")]
    pub fn shape_create_box(&mut self, size: Vector<Real>, handle: ShapeHandle) {
        let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
        self.insert_shape(shape, handle);
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_box(&mut self, size: Vector<Real>, handle: ShapeHandle) {
        let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y, 0.5 * size.z);
        self.insert_shape(shape, handle);
    }

    pub fn shape_get_box_size(&self, shape_handle: ShapeHandle) -> Vector<Real> {
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_cuboid() {
                return shape.half_extents;
            }
        }
        Vector::zeros()
    }

    pub fn shape_create_halfspace(
        &mut self,
        normal: Vector<Real>,
        distance: Real,
        handle: ShapeHandle,
    ) {
        let shape = SharedShape::halfspace(UnitVector::new_normalize(normal));
        let shape_position = Isometry::new(normal * distance, ANG_ZERO);
        let shapes_vec = vec![(shape_position, shape)];
        let shape_compound = SharedShape::compound(shapes_vec);
        self.insert_shape(shape_compound, handle);
    }

    pub fn shape_get_halfspace(&self, shape_handle: ShapeHandle) -> (Vector<Real>, Real) {
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_compound() {
                if let Some(shape) = shape.shapes().first() {
                    let normal = shape.0.translation.vector;
                    return (normal.normalize(), normal.magnitude());
                }
            }
        }
        (Vector::zeros(), 0.0)
    }

    pub fn shape_create_circle(&mut self, radius: Real, handle: ShapeHandle) {
        let shape = SharedShape::ball(radius);
        self.insert_shape(shape, handle);
    }

    pub fn shape_circle_get_radius(&self, shape_handle: ShapeHandle) -> Real {
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_ball() {
                return shape.radius;
            }
        }
        0.0
    }

    pub fn shape_create_capsule(&mut self, half_height: Real, radius: Real, handle: ShapeHandle) {
        let shape = SharedShape::capsule_y(half_height, radius);
        self.insert_shape(shape, handle);
    }

    pub fn shape_get_capsule(&self, shape_handle: ShapeHandle) -> (Real, Real) {
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_capsule() {
                return (shape.half_height(), shape.radius);
            }
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
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_cylinder() {
                return (shape.half_height, shape.radius);
            }
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
        use nalgebra::Vector3;
        let width = width as usize;
        let depth = depth as usize;
        // Create the height matrix from the input heights
        let original_heights = DMatrix::from_fn(width, depth, |i, j| heights[j * width + i]);
        // Transpose the matrix to swap width and depth
        let rotated_heights = original_heights.transpose();
        // The new dimensions after rotating the heightmap
        let new_width = depth;
        let new_depth = width;
        // Create the shape with the rotated dimensions
        let shape = SharedShape::heightfield_with_flags(
            rotated_heights,
            Vector3::new(new_depth as Real, 1.0, new_width as Real),
            HeightFieldFlags::FIX_INTERNAL_EDGES,
        );
        self.insert_shape(shape, handle)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_get_heightmap(&self, shape_handle: ShapeHandle) -> (DMatrix<Real>, i32, i32) {
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_heightfield() {
                let scale = shape.scale();
                let depth = scale.x as i32;
                let width = scale.z as i32;
                return (shape.heights().clone(), depth, width);
            }
        }
        (DMatrix::default(), 0, 0)
    }

    #[cfg(feature = "dim2")]
    pub fn shape_create_concave_polyline(
        &mut self,
        points: &Vec<Vector<Real>>,
        indices: Option<Vec<[u32; 2]>>,
        handle: ShapeHandle,
    ) {
        if points.len() == 0 {
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
        points: &Vec<Vector<Real>>,
        indices: Option<Vec<[u32; 3]>>,
        handle: ShapeHandle,
    ) {
        let points_vec = point_array_to_vec(points);
        let shape = SharedShape::trimesh_with_flags(
            points_vec,
            indices.unwrap(),
            TriMeshFlags::FIX_INTERNAL_EDGES,
        );
        self.insert_shape(shape.unwrap(), handle)
    }

    #[cfg(feature = "dim2")]
    pub fn shape_get_concave_polyline(
        &self,
        shape_handle: ShapeHandle,
    ) -> (&[Point<Real>], &[[u32; 2]]) {
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_polyline() {
                return (shape.vertices(), shape.indices());
            }
        }
        (&[], &[])
    }

    #[cfg(feature = "dim3")]
    pub fn shape_get_concave_polyline(
        &self,
        shape_handle: ShapeHandle,
    ) -> (&[Point<Real>], &[[u32; 3]]) {
        if let Some(shape) = self.get_shape(shape_handle) {
            if let Some(shape) = shape.as_trimesh() {
                return (shape.vertices(), shape.indices());
            }
        }
        (&[], &[])
    }

    pub fn shape_get_aabb(&self, handle: ShapeHandle) -> rapier::prelude::Aabb {
        if let Some(shape) = self.get_shape(handle) {
            return shape.compute_local_aabb();
        }
        godot_error!("Shape not found");
        rapier::prelude::Aabb::new_invalid()
    }

    pub fn shape_destroy(&mut self, shape_handle: ShapeHandle) {
        self.remove_shape(shape_handle)
    }
}
