use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::shapes::rapier_shape::*;
use crate::shapes::rapier_shape_base::RapierShapeBase;
pub struct RapierCylinderShape3D {
    base: RapierShapeBase,
}
impl RapierCylinderShape3D {
    pub fn create(id: RapierId, rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            base: RapierShapeBase::new(id, rid),
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

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        let height;
        let radius;
        match data.get_type() {
            VariantType::ARRAY => {
                let arr: Array<f32> = data.try_to().unwrap_or_default();
                if arr.len() != 2 {
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
                let dictionary: VarDictionary = data.try_to().unwrap_or_default();
                if let Some(in_height) = dictionary.get("height")
                    && let Some(in_radius) = dictionary.get("radius")
                {
                    if let Ok(in_height) = in_height.try_to::<real>() {
                        height = in_height;
                    } else {
                        godot_error!(
                            "RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}",
                            data
                        );
                        return;
                    }
                    if let Ok(in_radius) = in_radius.try_to::<real>() {
                        radius = in_radius;
                    } else {
                        godot_error!(
                            "RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}",
                            data
                        );
                        return;
                    }
                } else {
                    godot_error!(
                        "RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}",
                        data
                    );
                    return;
                }
            }
            _ => {
                godot_error!(
                    "RapierCylinderShape data must be a dictionary with 'height' and 'radius' keys. Got {}",
                    data
                );
                return;
            }
        }
        physics_engine.shape_create_cylinder(height / 2.0, radius, self.base.get_id());
        self.base.reset_aabb(physics_engine);
    }

    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let (half_height, radius) = physics_engine.shape_get_cylinder(self.base.get_id());
        Vector2::new(radius, half_height * 2.0).to_variant()
    }
}
#[cfg(feature = "test")]
mod tests {
    use godot::prelude::*;

    use super::*;
    use crate::servers::rapier_physics_singleton::PhysicsShapes;
    use crate::servers::rapier_physics_singleton::physics_data;
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
            RapierCylinderShape3D::create(0, rid, &mut physics_shapes);
            assert!(physics_shapes.contains_key(&rid));
            match physics_shapes.get(&rid) {
                Some(RapierShape::RapierCylinderShape3D(_)) => {}
                _ => panic!("Shape was not inserted correctly"),
            }
            let cylinder_shape = physics_shapes.get_mut(&rid).unwrap();
            assert_eq!(cylinder_shape.get_type(), ShapeType::CYLINDER);
            assert!(cylinder_shape.allows_one_way_collision());
        }

        #[func]
        fn test_set_data_array() {
            let mut cylinder_shape = RapierCylinderShape3D {
                base: RapierShapeBase::new(RapierId::default(), Rid::Invalid),
            };
            let mut arr = Array::default();
            arr.push(1.0);
            arr.push(0.5);
            cylinder_shape.set_data(arr.to_variant(), &mut physics_data().physics_engine);
            let data: Vector2 = cylinder_shape
                .get_data(&physics_data().physics_engine)
                .try_to()
                .unwrap();
            assert_eq!(data.x, 0.5); // radius
            assert_eq!(data.y, 1.0); // height
            cylinder_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
        }

        #[func]
        fn test_set_data_vector2() {
            let mut cylinder_shape = RapierCylinderShape3D {
                base: RapierShapeBase::new(RapierId::default(), Rid::Invalid),
            };
            let vec = Vector2::new(0.5, 1.0);
            cylinder_shape.set_data(vec.to_variant(), &mut physics_data().physics_engine);
            let data: Vector2 = cylinder_shape
                .get_data(&physics_data().physics_engine)
                .try_to()
                .unwrap();
            assert_eq!(data.x, 0.5); // radius
            assert_eq!(data.y, 1.0); // height
            cylinder_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
        }

        #[func]
        fn test_set_data_dictionary() {
            let rid = Rid::new(123);
            let mut cylinder_shape = RapierCylinderShape3D {
                base: RapierShapeBase::new(0, rid),
            };
            let mut dict = VarDictionary::new();
            let _ = dict.insert("height", 1.0);
            let _ = dict.insert("radius", 0.5);
            cylinder_shape.set_data(dict.to_variant(), &mut physics_data().physics_engine);
            let data: Vector2 = cylinder_shape
                .get_data(&physics_data().physics_engine)
                .try_to()
                .unwrap();
            assert_eq!(data.x, 0.5); // radius
            assert_eq!(data.y, 1.0); // height
            cylinder_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
        }
    }
}
