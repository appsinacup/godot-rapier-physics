#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsRids;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
#[cfg(feature = "dim2")]
use crate::types::PackedFloatArray;
use crate::types::PackedVectorArray;
pub struct RapierConvexPolygonShape {
    base: RapierShapeBase,
}
impl RapierConvexPolygonShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierConvexPolygonShape(shape));
    }
}
impl RapierConvexPolygonShape {
    fn create_rapier_shape(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        points: &PackedVectorArray,
    ) -> ShapeHandle {
        if points.len() >= 3 {
            let mut rapier_points = Vec::with_capacity(points.len());
            for point in points.as_slice() {
                rapier_points.push(vector_to_rapier(*point));
            }
            physics_engine.shape_create_convex_polyline(&rapier_points)
        } else {
            godot_error!("ConvexPolygon must have at least three point");
            ShapeHandle::default()
        }
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

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn set_data(
        &mut self,
        data: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    ) {
        let points;
        match data.get_type() {
            VariantType::PACKED_VECTOR2_ARRAY | VariantType::PACKED_VECTOR3_ARRAY => {
                if let Ok(arr) = data.try_to::<PackedVectorArray>() {
                    let size = arr.len();
                    if size < 3 {
                        godot_error!("ConvexPolygon must have at least three point");
                        return;
                    }
                    points = arr;
                } else {
                    godot_error!("ConvexPolygon data must be a PackedVectorArray");
                    return;
                }
            }
            #[cfg(feature = "dim2")]
            VariantType::PACKED_FLOAT64_ARRAY | VariantType::PACKED_FLOAT32_ARRAY => {
                if let Ok(arr) = data.try_to::<PackedFloatArray>() {
                    let size = arr.len() / 4;
                    if size < 3 {
                        godot_error!("ConvexPolygon must have at least three point");
                        return;
                    }
                    let mut new_points = PackedVectorArray::new();
                    for i in 0..size {
                        let idx = i << 2;
                        // skip normals
                        new_points.push(Vector2::new(arr[idx] as real, arr[idx + 1] as real));
                    }
                    points = new_points;
                } else {
                    godot_error!("ConvexPolygon data must be a PackedFloatArray");
                    return;
                }
            }
            _ => {
                godot_error!(
                    "ConvexPolygon data must be a PackedVectorArray or a PackedFloatArray"
                );
                return;
            }
        }
        if points.len() < 3 {
            godot_error!("ConvexPolygon must have at least three point");
            return;
        }
        let handle = self.create_rapier_shape(physics_engine, &points);
        if handle == ShapeHandle::default() {
            godot_error!("ConvexPolygon failed to create shape");
            return;
        }
        let new_points = physics_engine.shape_get_convex_polyline_points(handle);
        if new_points.len() != points.len() {
            godot_warn!(
                "ConvexPolygon shape points changed from size {} to {}",
                points.len(),
                new_points.len(),
            );
        }
        self.base
            .set_handle_and_reset_aabb(handle, physics_engine, physics_rids);
    }

    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let points = physics_engine.shape_get_convex_polyline_points(self.base.get_handle());
        let mut result_points = PackedVectorArray::new();
        for point in points.iter() {
            result_points.push(vector_to_godot(*point));
        }
        result_points.to_variant()
    }
}
#[cfg(feature = "test")]
mod tests {
    use godot::prelude::*;

    use super::*;
    use crate::servers::rapier_physics_singleton::physics_data;
    use crate::servers::rapier_physics_singleton::PhysicsShapes;
    use crate::shapes::rapier_shape::IRapierShape;
    use crate::types::*;
    #[derive(GodotClass)]
    #[class(base=Object, init)]
    pub struct RapierConvexPolygonShapeTests {}
    #[godot_api]
    impl RapierConvexPolygonShapeTests {
        #[func]
        fn test_create() {
            let mut physics_shapes = PhysicsShapes::new();
            let rid = Rid::new(123);
            RapierConvexPolygonShape::create(rid, &mut physics_shapes);
            assert!(physics_shapes.contains_key(&rid));
            match physics_shapes.get(&rid) {
                Some(RapierShape::RapierConvexPolygonShape(_)) => {}
                _ => panic!("Shape was not inserted correctly"),
            }
            let convex_shape = physics_shapes.get(&rid).unwrap();
            assert_eq!(convex_shape.get_type(), ShapeType::CONVEX_POLYGON);
            assert!(convex_shape.allows_one_way_collision());
        }

        #[cfg(feature = "dim2")]
        #[func]
        fn test_set_data() {
            let rid = Rid::new(123);
            let mut convex_shape = RapierConvexPolygonShape {
                base: RapierShapeBase::new(rid),
            };
            let arr = PackedVectorArray::from(vec![
                Vector::new(1.0, 1.0),
                Vector::new(4.0, 1.0),
                Vector::new(5.0, 3.0),
                Vector::new(3.0, 5.0),
            ]);
            convex_shape.set_data(
                arr.to_variant(),
                &mut physics_data().physics_engine,
                &mut physics_data().rids,
            );
            assert!(convex_shape.get_base().is_valid());
            let data: PackedVectorArray = convex_shape
                .get_data(&physics_data().physics_engine)
                .try_to()
                .unwrap();
            assert_eq!(data.len(), 4);
            assert_eq!(data[0], Vector::new(1.0, 1.0));
            assert_eq!(data[1], Vector::new(4.0, 1.0));
            assert_eq!(data[2], Vector::new(5.0, 3.0));
            assert_eq!(data[3], Vector::new(3.0, 5.0));
            convex_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine, &mut physics_data().rids);
            assert!(!convex_shape.get_base().is_valid());
        }

        #[cfg(feature = "dim3")]
        #[func]
        fn test_set_data() {
            let rid = Rid::new(123);
            let mut convex_shape = RapierConvexPolygonShape {
                base: RapierShapeBase::new(rid),
            };
            let arr = PackedVectorArray::from(vec![
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(2.0, 2.0, 2.0),
            ]);
            convex_shape.set_data(
                arr.to_variant(),
                &mut physics_data().physics_engine,
                &mut physics_data().rids,
            );
            let data: PackedVectorArray = convex_shape
                .get_data(&physics_data().physics_engine)
                .try_to()
                .unwrap();
            assert_eq!(data.len(), 3);
            // Order is changed on points
            assert_eq!(data[1], Vector::splat(0.0));
            assert_eq!(data[2], Vector3::new(1.0, 0.0, 0.0));
            assert_eq!(data[0], Vector3::new(2.0, 2.0, 2.0));
            assert!(convex_shape.get_base().is_valid());
            convex_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine, &mut physics_data().rids);
            assert!(!convex_shape.get_base().is_valid());
        }
    }
}
