#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::PackedFloatArray;
use crate::PackedVectorArray;
use crate::Rect;
use crate::Vector;
pub struct RapierConvexPolygonShape {
    points: PackedVectorArray,
    base: RapierShapeBase,
}
impl RapierConvexPolygonShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            points: PackedVectorArray::new(),
            base: RapierShapeBase::new(rid),
        }
    }

    fn compute_aabb(&self, scale: Vector) -> Rect {
        let mut aabb_new = Rect::new(Vector::ZERO, Vector::ZERO);
        for point in self.points.as_slice() {
            aabb_new = aabb_new.expand(*point * scale);
        }
        aabb_new
    }
}
impl IRapierShape for RapierConvexPolygonShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CONVEX_POLYGON
    }

    #[cfg(feature = "dim2")]
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> f32 {
        if self.points.len() < 3 {
            return 0.0;
        }
        let aabb_new = self.compute_aabb(scale);
        mass * aabb_new.size.dot(aabb_new.size) / 12.0
    }

    #[cfg(feature = "dim3")]
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> Vector3 {
        if self.points.len() < 3 {
            return Vector3::ZERO;
        }
        // use bad AABB approximation
        let extents = self.compute_aabb(scale).size * 0.5;
        Vector3::new(
            (mass / 3.0) * (extents.y * extents.y + extents.z * extents.z),
            (mass / 3.0) * (extents.x * extents.x + extents.z * extents.z),
            (mass / 3.0) * (extents.x * extents.x + extents.y * extents.y),
        )
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        if self.points.len() >= 3 {
            let mut rapier_points = Vec::with_capacity(self.points.len());
            for point in self.points.as_slice() {
                rapier_points.push(vector_to_rapier(*point));
            }
            shape_create_convex_polyline(rapier_points)
        }
        else {
            godot_error!("ConvexPolygon must have at least three point");
            invalid_handle()
        }
    }

    fn set_data(&mut self, data: Variant) {
        match data.get_type() {
            VariantType::PACKED_VECTOR2_ARRAY | VariantType::PACKED_VECTOR3_ARRAY => {
                if let Ok(arr) = data.try_to::<PackedVectorArray>() {
                    let size = arr.len();
                    if size <= 0 {
                        return;
                    }
                    self.points = arr;
                }
            }
            #[cfg(feature = "dim2")]
            VariantType::PACKED_FLOAT64_ARRAY | VariantType::PACKED_FLOAT32_ARRAY => {
                if let Ok(arr) = data.try_to::<PackedFloatArray>() {
                    let size = arr.len() / 4;
                    if size <= 0 {
                        return;
                    }
                    self.points = PackedVectorArray::new();
                    for i in 0..size {
                        let idx = i << 2;
                        // skip normals
                        self.points
                            .push(Vector2::new(arr[idx] as real, arr[idx + 1] as real));
                    }
                }
            }
            _ => {
                godot_error!("Invalid shape data");
                return;
            }
        }
        if self.points.len() < 3 {
            godot_error!("ConvexPolygon must have at least three point");
            return;
        }
        let handle = self.create_rapier_shape();
        self.base.set_handle(handle, self.compute_aabb(Vector::ONE));
    }

    fn get_data(&self) -> Variant {
        self.points.to_variant()
    }

    fn get_handle(&mut self) -> Handle {
        self.base.get_handle()
    }
}
