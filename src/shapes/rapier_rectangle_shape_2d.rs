use crate::rapier_wrapper::handle::Handle;
use crate::rapier_wrapper::shape::shape_create_box;
use crate::shapes::rapier_shape_2d::{IRapierShape2D, RapierShapeBase2D};
use godot::engine::physics_server_2d::ShapeType;
use godot::prelude::*;

pub struct RapierRectangleShape2D {
    half_extents: Vector2,
    pub base: RapierShapeBase2D,
}

impl RapierRectangleShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            half_extents: Vector2::ZERO,
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierRectangleShape2D {
    fn get_base(&self) -> &RapierShapeBase2D {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierShapeBase2D {
        &mut self.base
    }
    fn get_type(&self) -> ShapeType {
        ShapeType::RECTANGLE
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32 {
        let he2 = self.half_extents * 2.0 * scale;
        mass * he2.dot(he2) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        let v = rapier::na::Vector2::new(self.half_extents.x * 2.0, self.half_extents.y * 2.0);
        shape_create_box(v)
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() == VariantType::VECTOR2 {
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
        if !self.base.get_handle().is_valid() {
            let handle = self.create_rapier_shape();
            self.base.set_handle(handle);
        }
        self.base.get_handle()
    }
}
