use crate::rapier2d::handle::{invalid_handle, Handle};
use crate::rapier2d::shape::{shape_create_convex_polyline, shape_destroy};
use crate::rapier2d::vector::Vector;
use crate::shapes::rapier_shape_2d::{IRapierShape2D, RapierShapeBase2D};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierSegmentShape2D {
    a: Vector2,
    b: Vector2,
    n: Vector2,
    handle: Handle,

    pub base: RapierShapeBase2D,
}

impl RapierSegmentShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            a: Vector2::ZERO,
            b: Vector2::ZERO,
            n: Vector2::ZERO,
            handle: invalid_handle(),

            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierSegmentShape2D {
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
            Vector::new(p1.x, p1.y),
            Vector::new(p2.x, p2.y),
            Vector::new(p3.x, p3.y),
            Vector::new(p4.x, p4.y),
        ];

        return shape_create_convex_polyline(rapier_points.to_vec());
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::SEGMENT
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() == VariantType::Rect2 {
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
        } else {
            godot_error!("Invalid data type for RapierSegmentShape2D");
        }
    }

    fn get_data(&self) -> Variant {
        let mut r = Rect2::new(self.a, self.b - self.a);
        r.size = self.b - self.a;
        return r.to_variant();
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32 {
        mass * (self.a.distance_to(self.b) * scale.length_squared()) / 12.0
    }

    fn get_rapier_shape(&mut self) -> Handle {
        if !self.handle.is_valid() {
            self.handle = self.create_rapier_shape();
        }
        self.handle
    }

    fn destroy_rapier_shape(&mut self) {
        if self.handle.is_valid() {
            shape_destroy(self.handle);
            self.handle = invalid_handle();
        }
    }
}

impl Drop for RapierSegmentShape2D {
    fn drop(&mut self) {
        self.destroy_rapier_shape();
    }
}
