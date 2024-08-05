use godot::classes::physics_server_2d::ShapeType;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierSegmentShape2D {
    a: Vector2,
    b: Vector2,
    base: RapierShapeBase,
}
impl RapierSegmentShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            a: Vector2::ZERO,
            b: Vector2::ZERO,
            base: RapierShapeBase::new(rid),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierShape for RapierSegmentShape2D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::SEGMENT
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        let p1 = self.a;
        let p2 = self.b;
        let rapier_points = [vector_to_rapier(p1), vector_to_rapier(p2)];
        physics_engine.shape_create_concave_polyline(&rapier_points.to_vec(), None)
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        if data.get_type() != VariantType::RECT2 {
            godot_error!("Invalid shape data");
            return;
        }
        let r: Rect2 = data.try_to().unwrap_or_default();
        self.a = r.position;
        self.b = r.size;
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle(handle, physics_engine);
    }

    fn get_data(&self) -> Variant {
        let r = Rect2::new(self.a, self.b);
        r.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
