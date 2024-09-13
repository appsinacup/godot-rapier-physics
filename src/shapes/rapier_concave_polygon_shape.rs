#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
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

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
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
    pub struct RapierConcavePolygonShapeTests {}
    #[godot_api]
    impl RapierConcavePolygonShapeTests {
        #[func]
        fn test_create() {
            let mut physics_shapes = PhysicsShapes::new();
            let rid = Rid::new(123);
            RapierConcavePolygonShape::create(rid, &mut physics_shapes);
            assert!(physics_shapes.contains_key(&rid));
            match physics_shapes.get(&rid) {
                Some(RapierShape::RapierConcavePolygonShape(_)) => {}
                _ => panic!("Shape was not inserted correctly"),
            }
        }

        #[func]
        fn test_get_type() {
            let rid = Rid::new(123);
            let concave_shape = RapierConcavePolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            assert_eq!(concave_shape.get_type(), ShapeType::CONCAVE_POLYGON);
        }

        #[func]
        fn test_allows_one_way_collision() {
            let rid = Rid::new(123);
            let concave_shape = RapierConcavePolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            assert!(concave_shape.allows_one_way_collision());
        }

        #[func]
        fn test_set_data_from_array() {
            let rid = Rid::new(123);
            let mut concave_shape = RapierConcavePolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            let arr: PackedVector2Array = PackedVector2Array::from(vec![
                Vector::splat(0.0),
                Vector::splat(1.0),
                Vector::splat(2.0),
            ]);
            concave_shape.set_data(arr.to_variant(), &mut physics_data().physics_engine);
            assert_eq!(concave_shape.points.len(), 3);
            assert!(concave_shape.get_base().is_valid());
            concave_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!concave_shape.get_base().is_valid());
        }

        #[func]
        fn test_set_data_from_dictionary() {
            let rid = Rid::new(123);
            let mut concave_shape = RapierConcavePolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            let mut dict = Dictionary::new();
            let arr = PackedVector3Array::from(vec![
                Vector::splat(0.0),
                Vector::splat(1.0),
                Vector::splat(2.0),
            ]);
            let _ = dict.insert("faces", arr);
            concave_shape.set_data(dict.to_variant(), &mut physics_data().physics_engine);
            assert_eq!(concave_shape.points.len(), 3);
            assert!(concave_shape.get_base().is_valid());
            concave_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!concave_shape.get_base().is_valid());
        }

        #[func]
        fn test_get_data() {
            let rid = Rid::new(123);
            let concave_shape = RapierConcavePolygonShape {
                points: PackedVectorArray::default(),
                base: RapierShapeBase::new(rid),
            };
            let data: PackedVector2Array = concave_shape.get_data().try_to().unwrap();
            assert_eq!(data.len(), 0);
        }
    }
}
