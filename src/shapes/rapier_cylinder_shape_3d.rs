use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::*;
use crate::shapes::rapier_shape_base::RapierShapeBase;
pub struct RapierCylinderShape3D {
    height: f32,
    radius: f32,
    base: RapierShapeBase,
}
impl RapierCylinderShape3D {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierCylinderShape3D(shape));
    }
}
impl IRapierShape for RapierCylinderShape3D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CYLINDER
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        physics_engine.shape_create_cylinder((self.height / 2.0) - self.radius, self.radius)
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
                if let Some(height) = dictionary.get("height")
                    && let Some(radius) = dictionary.get("radius")
                {
                    if let Ok(height) = height.try_to::<real>() {
                        self.height = height;
                    } else {
                        godot_error!("RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}", data);
                        return;
                    }
                    if let Ok(radius) = radius.try_to::<real>() {
                        self.radius = radius;
                    } else {
                        godot_error!("RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}", data);
                        return;
                    }
                } else {
                    godot_error!("RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}", data);
                    return;
                }
            }
            _ => {
                godot_error!("RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}", data);
                return;
            }
        }
        if self.radius >= self.height * 0.5 {
            self.radius = self.height * 0.5;
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    fn get_data(&self) -> Variant {
        Vector2::new(self.radius, self.height).to_variant()
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
    pub struct RapierCylinderShape3DTests {}
    #[godot_api]
    impl RapierCylinderShape3DTests {
        #[func]
        fn test_create() {
            let mut physics_shapes = PhysicsShapes::new();
            let rid = Rid::new(123);
            RapierCylinderShape3D::create(rid, &mut physics_shapes);
            assert!(physics_shapes.contains_key(&rid));
            match physics_shapes.get(&rid) {
                Some(RapierShape::RapierCylinderShape3D(_)) => {}
                _ => panic!("Shape was not inserted correctly"),
            }
        }

        #[func]
        fn test_get_type() {
            let rid = Rid::new(123);
            let cylinder_shape = RapierCylinderShape3D {
                height: 0.0,
                radius: 0.0,
                base: RapierShapeBase::new(rid),
            };
            assert_eq!(cylinder_shape.get_type(), ShapeType::CYLINDER);
        }

        #[func]
        fn test_allows_one_way_collision() {
            let rid = Rid::new(123);
            let cylinder_shape = RapierCylinderShape3D {
                height: 0.0,
                radius: 0.0,
                base: RapierShapeBase::new(rid),
            };
            assert!(cylinder_shape.allows_one_way_collision());
        }

        #[func]
        fn test_set_data_array() {
            let rid = Rid::new(123);
            let mut cylinder_shape = RapierCylinderShape3D {
                height: 0.0,
                radius: 0.0,
                base: RapierShapeBase::new(rid),
            };
            let mut arr = Array::default();
            arr.push(1.0);
            arr.push(0.5);
            cylinder_shape.set_data(arr.to_variant(), &mut physics_data().physics_engine);
            // Now use get_data to verify the set values
            let data: Vector2 = cylinder_shape.get_data().try_to().unwrap();
            assert_eq!(data.x, 0.5); // radius
            assert_eq!(data.y, 1.0); // height
            assert!(cylinder_shape.get_base().is_valid());
            cylinder_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!cylinder_shape.get_base().is_valid());
        }

        #[func]
        fn test_set_data_vector2() {
            let rid = Rid::new(123);
            let mut cylinder_shape = RapierCylinderShape3D {
                height: 0.0,
                radius: 0.0,
                base: RapierShapeBase::new(rid),
            };
            let vec = Vector2::new(0.5, 1.0);
            cylinder_shape.set_data(vec.to_variant(), &mut physics_data().physics_engine);
            // Now use get_data to verify the set values
            let data: Vector2 = cylinder_shape.get_data().try_to().unwrap();
            assert_eq!(data.x, 0.5); // radius
            assert_eq!(data.y, 1.0); // height
            assert!(cylinder_shape.get_base().is_valid());
            cylinder_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!cylinder_shape.get_base().is_valid());
        }

        #[func]
        fn test_set_data_dictionary() {
            let rid = Rid::new(123);
            let mut cylinder_shape = RapierCylinderShape3D {
                height: 0.0,
                radius: 0.0,
                base: RapierShapeBase::new(rid),
            };
            let mut dict = Dictionary::new();
            let _ = dict.insert("height", 1.0);
            let _ = dict.insert("radius", 0.5);
            cylinder_shape.set_data(dict.to_variant(), &mut physics_data().physics_engine);
            // Now use get_data to verify the set values
            let data: Vector2 = cylinder_shape.get_data().try_to().unwrap();
            assert_eq!(data.x, 0.5); // radius
            assert_eq!(data.y, 1.0); // height
            assert!(cylinder_shape.get_base().is_valid());
            cylinder_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!cylinder_shape.get_base().is_valid());
        }

        #[func]
        fn test_get_data() {
            let rid = Rid::new(123);
            let cylinder_shape = RapierCylinderShape3D {
                height: 1.0,
                radius: 0.5,
                base: RapierShapeBase::new(rid),
            };
            let data: Vector2 = cylinder_shape.get_data().try_to().unwrap();
            assert_eq!(data.x, 0.5);
            assert_eq!(data.y, 1.0);
        }
    }
}
