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
    base: RapierShapeBase,
}
impl RapierConcavePolygonShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierConcavePolygonShape(shape));
    }
}
impl RapierConcavePolygonShape {
    fn create_rapier_shape(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        points: &PackedVectorArray,
    ) -> ShapeHandle {
        let point_count = points.len();
        let mut rapier_points = Vec::with_capacity(point_count);
        for i in 0..point_count {
            rapier_points.push(vector_to_rapier(points[i]));
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

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        let points_local;
        match data.get_type() {
            #[cfg(feature = "dim3")]
            VariantType::DICTIONARY => {
                if let Ok(dictionary) = data.try_to::<Dictionary>()
                    && let Some(points) = dictionary.get("faces")
                    && let Ok(arr) = points.try_to::<PackedVector3Array>()
                {
                    let len = arr.len();
                    if len == 0 {
                        godot_error!("ConcavePolygon3D must have at least one face");
                        return;
                    }
                    if len % 3 != 0 {
                        godot_error!("ConcavePolygon3D must have a multiple of 3 number of points");
                        return;
                    }
                    points_local = arr;
                } else {
                    godot_error!("ConcavePolygon3D data must be a PackedVector3Array");
                    return;
                }
            }
            #[cfg(feature = "dim2")]
            VariantType::PACKED_VECTOR2_ARRAY => {
                if let Ok(arr) = data.try_to::<PackedVector2Array>() {
                    let len = arr.len();
                    if len == 0 {
                        godot_error!("ConcavePolygon2D must have at least two points");
                        return;
                    }
                    if len % 2 != 0 {
                        godot_error!("ConcavePolygon2D must have an even number of points");
                        return;
                    }
                    points_local = arr;
                } else {
                    godot_error!("ConcavePolygon2D data must be a PackedVector2Array");
                    return;
                }
            }
            _ => {
                // Handle dictionary with arrays
                godot_error!(
                    "ConcavePolygon data must be a PackedVector3Array or a PackedVector2Array"
                );
                return;
            }
        }
        let handle = self.create_rapier_shape(physics_engine, &points_local);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    #[cfg(feature = "dim2")]
    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let (points, indices) = physics_engine.shape_get_concave_polyline(self.base.get_handle());
        let mut arr = PackedVectorArray::new();
        for ind in indices {
            if let Some(point) = points.get(ind[0] as usize) {
                arr.push(Vector2::new(point.coords.x, point.coords.y));
            } else {
                godot_error!("ConcavePolygon index out of bounds");
                arr.clear();
                break;
            }
            if let Some(point) = points.get(ind[1] as usize) {
                arr.push(Vector2::new(point.coords.x, point.coords.y));
            } else {
                godot_error!("ConcavePolygon index out of bounds");
                arr.clear();
                break;
            }
        }
        arr.to_variant()
    }

    #[cfg(feature = "dim3")]
    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let (points, indices) = physics_engine.shape_get_concave_polyline(self.base.get_handle());
        let mut arr = PackedVectorArray::new();
        for ind in indices {
            if let Some(point) = points.get(ind[0] as usize) {
                arr.push(Vector3::new(point.coords.x, point.coords.y, point.coords.z));
            } else {
                godot_error!("ConcavePolygon index out of bounds");
                arr.clear();
                break;
            }
            if let Some(point) = points.get(ind[1] as usize) {
                arr.push(Vector3::new(point.coords.x, point.coords.y, point.coords.z));
            } else {
                godot_error!("ConcavePolygon index out of bounds");
                arr.clear();
                break;
            }
            if let Some(point) = points.get(ind[2] as usize) {
                arr.push(Vector3::new(point.coords.x, point.coords.y, point.coords.z));
            } else {
                godot_error!("ConcavePolygon index out of bounds");
                arr.clear();
                break;
            }
        }
        arr.to_variant()
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
            let concave_shape = physics_shapes.get(&rid).unwrap();
            assert_eq!(concave_shape.get_type(), ShapeType::CONCAVE_POLYGON);
            assert!(concave_shape.allows_one_way_collision());
        }

        #[cfg(feature = "dim2")]
        #[func]
        fn test_set_data() {
            let mut concave_shape = RapierConcavePolygonShape {
                base: RapierShapeBase::new(Rid::Invalid),
            };
            let arr = PackedVectorArray::from(vec![
                Vector::splat(0.0),
                Vector::splat(1.0),
                Vector::splat(2.0),
                Vector::splat(4.0),
            ]);
            concave_shape.set_data(arr.to_variant(), &mut physics_data().physics_engine);
            assert!(concave_shape.get_base().is_valid());
            let data: PackedVectorArray = concave_shape
                .get_data(&physics_data().physics_engine)
                .try_to()
                .unwrap();
            assert_eq!(data.len(), 4);
            assert_eq!(data[0], Vector::splat(0.0));
            assert_eq!(data[1], Vector::splat(1.0));
            assert_eq!(data[2], Vector::splat(2.0));
            assert_eq!(data[3], Vector::splat(4.0));
            concave_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!concave_shape.get_base().is_valid());
        }

        #[cfg(feature = "dim3")]
        #[func]
        fn test_set_data() {
            let mut concave_shape = RapierConcavePolygonShape {
                base: RapierShapeBase::new(Rid::Invalid),
            };
            let mut dict = Dictionary::new();
            let arr = PackedVectorArray::from(vec![
                Vector::splat(0.0),
                Vector::splat(1.0),
                Vector::splat(2.0),
                Vector::splat(3.0),
                Vector::splat(4.0),
                Vector::splat(5.0),
            ]);
            let _ = dict.insert("faces", arr);
            concave_shape.set_data(dict.to_variant(), &mut physics_data().physics_engine);
            let data: PackedVectorArray = concave_shape
                .get_data(&physics_data().physics_engine)
                .try_to()
                .unwrap();
            assert_eq!(data.len(), 6);
            assert_eq!(data[0], Vector::splat(0.0));
            assert_eq!(data[1], Vector::splat(1.0));
            assert_eq!(data[2], Vector::splat(2.0));
            assert_eq!(data[3], Vector::splat(3.0));
            assert_eq!(data[4], Vector::splat(4.0));
            assert_eq!(data[5], Vector::splat(5.0));
            assert!(concave_shape.get_base().is_valid());
            concave_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!concave_shape.get_base().is_valid());
        }
    }
}
