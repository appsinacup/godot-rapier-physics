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
    use nalgebra::Quaternion;
    use nalgebra::Translation3;
    let quaternion = transform.basis.to_quat();
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
        let shapes_vec = vec![(shape_position, shape)];
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

    #[cfg(feature = "dim3")]
    pub fn shape_create_heightmap(
        &mut self,
        heights: &[Real],
        width: i32,
        depth: i32,
    ) -> ShapeHandle {
        use nalgebra::Vector3;
        let width = width as usize;
        let depth = depth as usize;
        let heights = DMatrix::from_fn(width, depth, |i, j| heights[j * (width) + i]);
        let shape = SharedShape::heightfield_with_flags(
            heights,
            Vector3::new(depth as Real, 1.0, width as Real),
            HeightFieldFlags::FIX_INTERNAL_EDGES,
        );
        self.insert_shape(shape)
    }

    #[cfg(feature = "dim2")]
    pub fn shape_create_concave_polyline(
        &mut self,
        points: &Vec<Vector<Real>>,
        indices: Option<Vec<[u32; 2]>>,
    ) -> ShapeHandle {
        let points_vec = point_array_to_vec(points);
        let shape = SharedShape::polyline(points_vec, indices);
        self.insert_shape(shape)
    }

    #[cfg(feature = "dim3")]
    pub fn shape_create_concave_polyline(
        &mut self,
        points: &Vec<Vector<Real>>,
        indices: Option<Vec<[u32; 3]>>,
    ) -> ShapeHandle {
        let points_vec = point_array_to_vec(points);
        let shape = SharedShape::trimesh_with_flags(
            points_vec,
            indices.unwrap(),
            TriMeshFlags::FIX_INTERNAL_EDGES,
        );
        self.insert_shape(shape)
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
