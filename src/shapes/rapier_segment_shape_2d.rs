use godot::classes::physics_server_2d::ShapeType;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::types::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierSegmentShape2D {
    a: Vector2,
    b: Vector2,
    n: Vector2,
    base: RapierShapeBase,
}
impl RapierSegmentShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            a: Vector2::ZERO,
            b: Vector2::ZERO,
            n: Vector2::ZERO,
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

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> f32 {
        mass * (self.a.distance_to(self.b) * scale.length_squared()) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        let direction = self.b - self.a;
        let direction_normalized = vector_normalized(direction);
        let perpendicular = Vector2::new(-direction_normalized.y, direction_normalized.x);
        let height = 0.1;
        let p1 = self.a + perpendicular * height / 2.0;
        let p2 = self.a - perpendicular * height / 2.0;
        let p3 = self.b + perpendicular * height / 2.0;
        let p4 = self.b - perpendicular * height / 2.0;
        let rapier_points = [
            vector_to_rapier(p1),
            vector_to_rapier(p2),
            vector_to_rapier(p3),
            vector_to_rapier(p4),
        ];
        physics_engine.shape_create_convex_polyline(&rapier_points.to_vec())
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        if data.get_type() != VariantType::RECT2 {
            godot_error!("Invalid shape data");
            return;
        }
        let r: Rect2 = data.to();
        self.a = r.position;
        self.b = r.position + r.size;
        self.n = (self.b - self.a).orthogonal();
        let mut aabb = Rect2::new(self.a, self.b);
        if aabb.size.x == 0.0 {
            aabb.size.x = 0.001;
        }
        if aabb.size.y == 0.0 {
            aabb.size.y = 0.001;
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle(handle, aabb, physics_engine);
    }

    fn get_data(&self) -> Variant {
        let mut r = Rect2::new(self.a, self.b - self.a);
        r.size = self.b - self.a;
        r.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
