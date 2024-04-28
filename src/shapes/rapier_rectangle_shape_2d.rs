use crate::rapier2d::handle::{invalid_handle, Handle};
use crate::rapier2d::shape::{shape_create_box, shape_destroy};
use crate::rapier2d::vector::Vector;
use crate::shapes::rapier_shape_2d::{IRapierShape2D, RapierShapeBase2D};
use godot::engine::physics_server_2d::ShapeType;
use godot::prelude::*;

pub struct RapierRectangleShape2D {
    half_extents: Vector2,
    handle: Handle,

    pub base: RapierShapeBase2D,
}

impl RapierRectangleShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            half_extents: Vector2::ZERO,
            handle: invalid_handle(),
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierRectangleShape2D {
    fn get_type(&self) -> ShapeType {
        ShapeType::SEGMENT
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32 {
        let he2 = self.half_extents * 2.0 * scale;
        mass * he2.dot(he2) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        let v = Vector::new(self.half_extents.x * 2.0, self.half_extents.y * 2.0);
        shape_create_box(&v)
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() == VariantType::Vector2 {
            let v: Vector2 = data.to();
            self.half_extents = v;
            let mut aabb = Rect2::new(-self.half_extents, self.half_extents * 2.0);
            if aabb.size.x == 0.0 {
                aabb.size.x = 0.001;
            }
            if aabb.size.y == 0.0 {
                aabb.size.y = 0.001;
            }
            self.base.configure(aabb);
        } else {
            godot_error!("Invalid data type for RapierRectangleShape2D");
        }
    }

    fn get_data(&self) -> Variant {
        self.half_extents.to_variant()
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

impl Drop for RapierRectangleShape2D {
    fn drop(&mut self) {
        self.destroy_rapier_shape();
    }
}
