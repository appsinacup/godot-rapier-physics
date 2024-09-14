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
use crate::types::*;
pub struct RapierCircleShape {
    radius: real,
    base: RapierShapeBase,
}
impl RapierCircleShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierCircleShape(shape));
    }
}
impl IRapierShape for RapierCircleShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    #[cfg(feature = "dim2")]
    fn get_type(&self) -> ShapeType {
        ShapeType::CIRCLE
    }

    #[cfg(feature = "dim3")]
    fn get_type(&self) -> ShapeType {
        ShapeType::SPHERE
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        physics_engine.shape_create_circle(self.radius)
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        match data.get_type() {
            VariantType::FLOAT | VariantType::INT => {
                self.radius = variant_to_float(&data);
            }
            _ => {
                godot_error!(
                    "RapierCircleShape data must be a float or int. Got {}",
                    data
                );
                return;
            }
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    fn get_data(&self) -> Variant {
        self.radius.to_variant()
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
    #[class(base = Object, init)]
    pub struct RapierCircleShapeTests {}
    #[godot_api]
    impl RapierCircleShapeTests {
        #[func]
        fn test_create() {
            let mut physics_shapes = PhysicsShapes::new();
            let rid = Rid::new(123);
            RapierCircleShape::create(rid, &mut physics_shapes);
            assert!(physics_shapes.contains_key(&rid));
            match physics_shapes.get(&rid) {
                Some(RapierShape::RapierCircleShape(_)) => {}
                _ => panic!("Shape was not inserted correctly"),
            }
        }

        #[func]
        fn test_get_type() {
            let rid = Rid::new(123);
            let circle_shape = RapierCircleShape {
                radius: 1.0,
                base: RapierShapeBase::new(rid),
            };
            #[cfg(feature = "dim2")]
            assert_eq!(circle_shape.get_type(), ShapeType::CIRCLE);
            #[cfg(feature = "dim3")]
            assert_eq!(circle_shape.get_type(), ShapeType::SPHERE);
        }

        #[func]
        fn test_allows_one_way_collision() {
            let rid = Rid::new(123);
            let circle_shape = RapierCircleShape {
                radius: 1.0,
                base: RapierShapeBase::new(rid),
            };
            assert!(circle_shape.allows_one_way_collision());
        }

        #[func]
        fn test_set_data() {
            let rid = Rid::new(123);
            let mut circle_shape = RapierCircleShape {
                radius: 0.0,
                base: RapierShapeBase::new(rid),
            };
            let data = Variant::from(1.5);
            circle_shape.set_data(data, &mut physics_data().physics_engine);
            assert_eq!(circle_shape.radius, 1.5);
            assert!(circle_shape.get_base().is_valid());
            circle_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
            assert!(!circle_shape.get_base().is_valid());
        }

        #[func]
        fn test_get_data() {
            let rid = Rid::new(123);
            let circle_shape = RapierCircleShape {
                radius: 1.5,
                base: RapierShapeBase::new(rid),
            };
            let data: real = circle_shape.get_data().try_to().unwrap();
            assert_eq!(data, 1.5);
        }
    }
}
