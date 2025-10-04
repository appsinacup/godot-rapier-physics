#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::shapes::rapier_shape::*;
use crate::shapes::rapier_shape_base::RapierShapeBase;
use crate::types::*;
pub struct RapierCircleShape {
    base: RapierShapeBase,
}
impl RapierCircleShape {
    pub fn create(id: RapierId, rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            base: RapierShapeBase::new(id, rid),
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

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        let radius = variant_to_float(&data);
        if radius < 0.0 {
            godot_error!("RapierCircleShape radius must be positive. Got {}", radius);
            return;
        }
        physics_engine.shape_create_circle(radius, self.base.get_id());
        self.base.reset_aabb(physics_engine)
    }

    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        physics_engine
            .shape_circle_get_radius(self.base.get_id())
            .to_variant()
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
    #[class(base = Object, init)]
    pub struct RapierCircleShapeTests {}
    #[godot_api]
    impl RapierCircleShapeTests {
        #[func]
        fn test_create() {
            let mut physics_shapes = PhysicsShapes::new();
            let rid = Rid::new(123);
            RapierCircleShape::create(0, rid, &mut physics_shapes);
            assert!(physics_shapes.contains_key(&rid));
            match physics_shapes.get(&rid) {
                Some(RapierShape::RapierCircleShape(_)) => {}
                _ => panic!("Shape was not inserted correctly"),
            }
            let circle_shape = physics_shapes.get(&rid).unwrap();
            #[cfg(feature = "dim2")]
            assert_eq!(circle_shape.get_type(), ShapeType::CIRCLE);
            #[cfg(feature = "dim3")]
            assert_eq!(circle_shape.get_type(), ShapeType::SPHERE);
            assert!(circle_shape.allows_one_way_collision());
        }

        #[func]
        fn test_set_data() {
            let mut circle_shape = RapierCircleShape {
                base: RapierShapeBase::new(RapierId::default(), Rid::Invalid),
            };
            let data = Variant::from(1.5);
            circle_shape.set_data(data, &mut physics_data().physics_engine);
            assert_eq!(
                circle_shape.get_data(&physics_data().physics_engine),
                1.5.to_variant()
            );
            circle_shape
                .get_mut_base()
                .destroy_shape(&mut physics_data().physics_engine);
        }
    }
}
