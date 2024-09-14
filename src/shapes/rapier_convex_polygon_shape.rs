#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
#[cfg(feature = "dim2")]
use crate::types::PackedFloatArray;
use crate::types::PackedVectorArray;
pub struct RapierConvexPolygonShape {
    points: PackedVectorArray,
    base: RapierShapeBase,
}
impl RapierConvexPolygonShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            points: PackedVectorArray::new(),
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierConvexPolygonShape(shape));
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

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        if self.points.len() >= 3 {
            let mut rapier_points = Vec::with_capacity(self.points.len());
            for point in self.points.as_slice() {
                rapier_points.push(vector_to_rapier(*point));
            }
            physics_engine.shape_create_convex_polyline(&rapier_points)
        } else {
            godot_error!("ConvexPolygon must have at least three point");
            ShapeHandle::default()
        }
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        match data.get_type() {
            VariantType::PACKED_VECTOR2_ARRAY | VariantType::PACKED_VECTOR3_ARRAY => {
                if let Ok(arr) = data.try_to::<PackedVectorArray>() {
                    let size = arr.len();
                    if size < 3 {
                        godot_error!("ConvexPolygon must have at least three point");
                        return;
                    }
                    self.points = arr;
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
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    fn get_data(&self) -> Variant {
        self.points.to_variant()
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
        }

        #[func]
        fn test_get_type() {
            let rid = Rid::new(123);
            let convex_shape = RapierConvexPolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            assert_eq!(convex_shape.get_type(), ShapeType::CONVEX_POLYGON);
        }

        #[func]
        fn test_allows_one_way_collision() {
            let rid = Rid::new(123);
            let convex_shape = RapierConvexPolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            assert!(convex_shape.allows_one_way_collision());
        }

        #[cfg(feature = "dim2")]
        #[func]
        fn test_set_data() {
            let rid = Rid::new(123);
            let mut convex_shape = RapierConvexPolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            let arr = PackedVectorArray::from(vec![
                Vector::splat(0.0),
                Vector::splat(1.0),
                Vector::splat(2.0),
            ]);
            convex_shape.set_data(arr.to_variant(), &mut physics_data().physics_engine);
            let data: PackedVectorArray = convex_shape.get_data().try_to().unwrap();
            assert_eq!(data.len(), 3);
            assert_eq!(data[0], Vector::splat(0.0));
            assert_eq!(data[1], Vector::splat(1.0));
            assert_eq!(data[2], Vector::splat(2.0));
        }

        #[cfg(feature = "dim3")]
        #[func]
        fn test_set_data() {
            let rid = Rid::new(123);
            let mut convex_shape = RapierConvexPolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            let arr = PackedVectorArray::from(vec![
                Vector::splat(0.0),
                Vector::splat(1.0),
                Vector::splat(2.0),
                Vector::splat(3.0),
            ]);
            convex_shape.set_data(arr.to_variant(), &mut physics_data().physics_engine);
            let data: PackedVectorArray = convex_shape.get_data().try_to().unwrap();
            assert_eq!(data.len(), 4);
            assert_eq!(data[0], Vector::splat(0.0));
            assert_eq!(data[1], Vector::splat(1.0));
            assert_eq!(data[2], Vector::splat(2.0));
            assert_eq!(data[3], Vector::splat(3.0));
        }

        #[func]
        fn test_get_data() {
            let rid = Rid::new(123);
            let convex_shape = RapierConvexPolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            let data: PackedVectorArray = convex_shape.get_data().try_to().unwrap();
            assert_eq!(data.len(), 0);
        }
    }
}
