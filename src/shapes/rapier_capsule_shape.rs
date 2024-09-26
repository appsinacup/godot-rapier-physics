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
pub struct RapierCapsuleShape {
    base: RapierShapeBase,
}
impl RapierCapsuleShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
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

    fn set_data(
        &mut self,
        data: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    ) {
        let height;
        let radius;
        match data.get_type() {
            VariantType::ARRAY => {
                let arr: Array<real> = data.try_to().unwrap_or_default();
                if arr.len() != 2 {
                    godot_error!(
                        "RapierCapsuleShape data must be an array of 2 elements. Got {}",
                        data
                    );
                    return;
                }
                height = arr.at(0);
                radius = arr.at(1);
            }
            VariantType::VECTOR2 => {
                let vector_data: Vector2 = data.try_to().unwrap_or_default();
                height = vector_data.y;
                radius = vector_data.x;
            }
            VariantType::DICTIONARY => {
                let dictionary: Dictionary = data.try_to().unwrap_or_default();
                if !dictionary.contains_key("length") && !dictionary.contains_key("height") {
                    godot_error!("RapierCapsuleShape data must be a dictionary with 'length' and 'height' keys. Got {}", data);
                    return;
                }
                if let Some(in_height) = dictionary.get("height")
                    && let Ok(in_height) = in_height.try_to()
                {
                    height = in_height;
                } else {
                    godot_error!("RapierCapsuleShape data must be a dictionary with 'length' and 'height' keys. Got {}", data);
                    return;
                }
                if let Some(in_radius) = dictionary.get("radius")
                    && let Ok(in_radius) = in_radius.try_to()
                {
                    radius = in_radius;
                } else {
                    godot_error!("RapierCapsuleShape data must be a dictionary with 'length' and 'height' keys. Got {}", data);
                    return;
                }
            }
            _ => {
                godot_error!(
                    "RapierCapsuleShape data must be an array or a dictionary. Got {}",
                    data
                );
                return;
            }
        }
        if height <= 0.0 {
            godot_error!("RapierCapsuleShape height must be positive. Got {}", height);
            return;
        }
        if radius <= 0.0 {
            godot_error!("RapierCapsuleShape radius must be positive. Got {}", radius);
            return;
        }
        let handle = physics_engine.shape_create_capsule((height / 2.0) - radius, radius);
        self.base
            .set_handle_and_reset_aabb(handle, physics_engine, physics_rids);
    }

    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let (height, radius) = physics_engine.shape_get_capsule(self.base.get_handle());
        Vector2::new((height + radius) * 2.0, radius).to_variant()
    }
}
#[cfg(feature = "test")]
mod tests {
    use godot::prelude::*;

    use super::*;
    use crate::servers::rapier_physics_singleton::physics_data;
    use crate::servers::rapier_physics_singleton::PhysicsShapes;
    use crate::shapes::rapier_shape::IRapierShape;
    #[derive(GodotClass)]
    #[class(base=Object, init)]
    pub struct RapierCapsuleShapeTests {}
    #[godot_api]
    impl RapierCapsuleShapeTests {
        #[func]
        fn test_create() {
            let mut physics_shapes = PhysicsShapes::new();
            let rid = Rid::new(123);
            RapierCapsuleShape::create(rid, &mut physics_shapes);
            assert!(physics_shapes.contains_key(&rid));
            match physics_shapes.get(&rid) {
                Some(RapierShape::RapierCapsuleShape(_)) => {}
                _ => panic!("Shape was not inserted correctly"),
            }
            let capsule_shape = physics_shapes.get(&rid).unwrap();
            assert_eq!(capsule_shape.get_type(), ShapeType::CAPSULE);
            assert!(capsule_shape.allows_one_way_collision());
        }

        #[func]
        fn test_set_data_from_array() {
            let mut capsule_shape = RapierCapsuleShape {
                base: RapierShapeBase::new(Rid::Invalid),
            };
            let arr: Array<real> = array![3.0, 1.0];
            capsule_shape.set_data(
                arr.to_variant(),
                &mut physics_data().physics_engine,
                &mut physics_data().rids,
            );
            let data = capsule_shape.get_data(&physics_data().physics_engine);
            let data_vector = data.try_to::<Vector2>().unwrap();
            assert_eq!(data_vector.x, 3.0);
            assert_eq!(data_vector.y, 1.0);
            assert!(capsule_shape.get_base().is_valid());
            capsule_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine, &mut physics_data().rids);
            assert!(!capsule_shape.get_base().is_valid());
        }

        #[func]
        fn test_set_data_from_vector2() {
            let mut capsule_shape = RapierCapsuleShape {
                base: RapierShapeBase::new(Rid::Invalid),
            };
            let vector2_data = Vector2::new(1.0, 2.0);
            capsule_shape.set_data(
                vector2_data.to_variant(),
                &mut physics_data().physics_engine,
                &mut physics_data().rids,
            );
            assert!(capsule_shape.get_base().is_valid());
            let data = capsule_shape.get_data(&physics_data().physics_engine);
            let data_vector = data.try_to::<Vector2>().unwrap();
            assert_eq!(data_vector.x, 2.0);
            assert_eq!(data_vector.y, 1.0);
            capsule_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine, &mut physics_data().rids);
            assert!(!capsule_shape.get_base().is_valid());
        }

        #[func]
        fn test_set_data_from_dictionary() {
            let mut capsule_shape = RapierCapsuleShape {
                base: RapierShapeBase::new(Rid::Invalid),
            };
            let mut dict = Dictionary::new();
            let _ = dict.insert("radius", 1.0);
            let _ = dict.insert("height", 2.0);
            capsule_shape.set_data(
                dict.to_variant(),
                &mut physics_data().physics_engine,
                &mut physics_data().rids,
            );
            assert!(capsule_shape.get_base().is_valid());
            let data = capsule_shape.get_data(&physics_data().physics_engine);
            let data_vector = data.try_to::<Vector2>().unwrap();
            assert_eq!(data_vector.x, 2.0);
            assert_eq!(data_vector.y, 1.0);
            capsule_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine, &mut physics_data().rids);
            assert!(!capsule_shape.get_base().is_valid());
        }
    }
}
