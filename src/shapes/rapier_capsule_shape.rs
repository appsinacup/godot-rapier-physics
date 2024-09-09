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
pub struct RapierCapsuleShape {
    height: f32,
    radius: f32,
    base: RapierShapeBase,
}
impl RapierCapsuleShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierCapsuleShape(shape));
    }
}
impl IRapierShape for RapierCapsuleShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CAPSULE
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        physics_engine.shape_create_capsule((self.height / 2.0) - self.radius, self.radius)
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        match data.get_type() {
            VariantType::ARRAY => {
                let arr: Array<f32> = data.try_to().unwrap_or_default();
                if arr.len() != 2 {
                    return;
                }
                self.height = arr.at(0);
                self.radius = arr.at(1);
            }
            VariantType::VECTOR2 => {
                let vector_data: Vector2 = data.try_to().unwrap_or_default();
                self.height = vector_data.y;
                self.radius = vector_data.x;
            }
            VariantType::DICTIONARY => {
                let dictionary: Dictionary = data.try_to().unwrap_or_default();
                if !dictionary.contains_key("length") && !dictionary.contains_key("height") {
                    godot_error!("Invalid shape data.");
                    return;
                }
                if let Some(height) = dictionary.get("height") {
                    if let Ok(height) = height.try_to::<real>() {
                        self.height = height;
                    }
                }
                if let Some(radius) = dictionary.get("radius") {
                    if let Ok(radius) = radius.try_to::<real>() {
                        self.radius = radius;
                    }
                }
            }
            _ => {
                godot_error!("Invalid shape data");
                return;
            }
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    fn get_data(&self) -> Variant {
        Vector2::new(self.radius, self.height).to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
#[cfg(test)]
mod tests {
    use godot::prelude::*;

    use super::*;
    use crate::servers::rapier_physics_singleton::physics_data;
    use crate::servers::rapier_physics_singleton::PhysicsShapes;
    use crate::shapes::rapier_shape::IRapierShape;
    #[test]
    fn test_create_capsule_shape() {
        let mut physics_shapes = PhysicsShapes::new();
        let rid = Rid::new(123);
        RapierCapsuleShape::create(rid, &mut physics_shapes);
        assert!(physics_shapes.contains_key(&rid));
        match physics_shapes.get(&rid) {
            Some(RapierShape::RapierCapsuleShape(_)) => {}
            _ => panic!("Shape was not inserted correctly"),
        }
    }
    #[test]
    fn test_get_type() {
        let rid = Rid::new(123);
        let capsule_shape = RapierCapsuleShape {
            height: 2.0,
            radius: 1.0,
            base: RapierShapeBase::new(rid),
        };
        assert_eq!(capsule_shape.get_type(), ShapeType::CAPSULE);
    }
    #[test]
    fn test_allows_one_way_collision() {
        let rid = Rid::new(123);
        let capsule_shape = RapierCapsuleShape {
            height: 2.0,
            radius: 1.0,
            base: RapierShapeBase::new(rid),
        };
        assert!(capsule_shape.allows_one_way_collision());
    }
    #[test]
    fn test_set_data_from_array() {
        let rid = Rid::new(123);
        let mut capsule_shape = RapierCapsuleShape {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        };
        let arr = array![Variant::from(2.0), Variant::from(1.0)];
        capsule_shape.set_data(arr.to_variant(), &mut physics_data().physics_engine);
        assert_eq!(capsule_shape.height, 2.0);
        assert_eq!(capsule_shape.radius, 1.0);
    }
    #[test]
    fn test_set_data_from_vector2() {
        let rid = Rid::new(123);
        let mut capsule_shape = RapierCapsuleShape {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        };
        let vector2_data = Vector2::new(1.0, 2.0);
        capsule_shape.set_data(
            vector2_data.to_variant(),
            &mut physics_data().physics_engine,
        );
        assert_eq!(capsule_shape.height, 2.0);
        assert_eq!(capsule_shape.radius, 1.0);
    }
    #[test]
    fn test_set_data_from_dictionary() {
        let rid = Rid::new(123);
        let mut capsule_shape = RapierCapsuleShape {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        };
        let mut dict = Dictionary::new();
        let _ = dict.insert("height", 2.0);
        let _ = dict.insert("radius", 1.0);
        capsule_shape.set_data(dict.to_variant(), &mut physics_data().physics_engine);
        assert_eq!(capsule_shape.height, 2.0);
        assert_eq!(capsule_shape.radius, 1.0);
    }
    #[test]
    fn test_get_data() {
        let rid = Rid::new(123);
        let capsule_shape = RapierCapsuleShape {
            height: 2.0,
            radius: 1.0,
            base: RapierShapeBase::new(rid),
        };
        let data: Vector2 = capsule_shape.get_data().try_to().unwrap();
        assert_eq!(data.x, 1.0);
        assert_eq!(data.y, 2.0);
    }
}
