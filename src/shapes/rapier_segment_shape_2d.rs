use crate::rapier_wrapper::handle::Handle;
use crate::rapier_wrapper::shape::shape_create_convex_polyline;
use crate::shapes::rapier_shape_2d::{IRapierShape2D, RapierShapeBase2D};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierSegmentShape2D {
    a: Vector2,
    b: Vector2,
    n: Vector2,
    pub base: RapierShapeBase2D,
}

impl RapierSegmentShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            a: Vector2::ZERO,
            b: Vector2::ZERO,
            n: Vector2::ZERO,
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierSegmentShape2D {
    fn get_base(&self) -> &RapierShapeBase2D {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierShapeBase2D {
        &mut self.base
    }
    fn get_type(&self) -> ShapeType {
        ShapeType::SEGMENT
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32 {
        mass * (self.a.distance_to(self.b) * scale.length_squared()) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        let direction = self.b - self.a;
        let direction_normalized = direction.normalized();
        let perpendicular = Vector2::new(-direction_normalized.y, direction_normalized.x);
        let height = 0.1;

        let p1 = self.a + perpendicular * height / 2.0;
        let p2 = self.a - perpendicular * height / 2.0;
        let p3 = self.b + perpendicular * height / 2.0;
        let p4 = self.b - perpendicular * height / 2.0;

        let rapier_points = [
            rapier::na::Vector2::new(p1.x, p1.y),
            rapier::na::Vector2::new(p2.x, p2.y),
            rapier::na::Vector2::new(p3.x, p3.y),
            rapier::na::Vector2::new(p4.x, p4.y),
        ];

        shape_create_convex_polyline(rapier_points.to_vec())
    }

    fn set_data(&mut self, data: Variant) {
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
        self.base.configure(aabb);
    }

    fn get_data(&self) -> Variant {
        let mut r = Rect2::new(self.a, self.b - self.a);
        r.size = self.b - self.a;
        r.to_variant()
    }

    fn get_rapier_shape(&mut self) -> Handle {
        if !self.base.get_handle().is_valid() {
            let handle = self.create_rapier_shape();
            self.base.set_handle(handle);
        }
        self.base.get_handle()
    }
}
