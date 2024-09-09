#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsRids;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::*;
use crate::shapes::rapier_shape_base::RapierShapeBase;
use crate::types::PackedVectorArray;
pub struct RapierConcavePolygonShape {
    points: PackedVectorArray,
    base: RapierShapeBase,
}
impl RapierConcavePolygonShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            points: PackedVectorArray::default(),
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierConcavePolygonShape(shape));
    }
}
impl IRapierShape for RapierConcavePolygonShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CONCAVE_POLYGON
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        let point_count = self.points.len();
        let mut rapier_points = Vec::with_capacity(point_count);
        for i in 0..point_count {
            rapier_points.push(vector_to_rapier(self.points[i]));
        }
        let mut segments = Vec::new();
        #[cfg(feature = "dim2")]
        for i in (0..point_count).step_by(2) {
            let s = [(i) as u32, (i + 1) as u32];
            segments.push(s);
        }
        #[cfg(feature = "dim3")]
        for i in (0..point_count).step_by(3) {
            let s = [(i) as u32, (i + 1) as u32, (i + 2) as u32];
            segments.push(s);
        }
        physics_engine.shape_create_concave_polyline(&rapier_points, Some(segments))
    }

    fn set_data(
        &mut self,
        data: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    ) {
        match data.get_type() {
            #[cfg(feature = "dim3")]
            VariantType::DICTIONARY => {
                if let Ok(dictionary) = data.try_to::<Dictionary>() {
                    if let Some(points) = dictionary.get("faces")
                        && let Ok(arr) = points.try_to::<PackedVector3Array>()
                    {
                        let len = arr.len();
                        if len == 0 {
                            return;
                        }
                        if len % 3 != 0 {
                            godot_error!(
                                "ConcavePolygon3D must have a multiple of 3 number of points"
                            );
                            return;
                        }
                        self.points = arr;
                    }
                }
            }
            #[cfg(feature = "dim2")]
            VariantType::PACKED_VECTOR2_ARRAY => {
                if let Ok(arr) = data.try_to::<PackedVector2Array>() {
                    let len = arr.len();
                    if len == 0 {
                        return;
                    }
                    if len % 2 != 0 {
                        godot_error!("ConcavePolygon2D must have an even number of points");
                        return;
                    }
                    self.points = arr;
                }
            }
            _ => {
                // Handle dictionary with arrays
                godot_error!("Invalid shape data");
                return;
            }
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base
            .set_handle_and_reset_aabb(handle, physics_engine, physics_rids);
    }

    fn get_data(&self) -> Variant {
        self.points.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
