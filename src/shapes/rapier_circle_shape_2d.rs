use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::{
    rapier2d::{
        handle::{invalid_handle, Handle},
        shape::{shape_create_circle, shape_destroy},
    },
    shapes::rapier_shape_2d::RapierShapeBase2D,
};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierCircleShape2D {
    radius: f32,
    handle: Handle,

    pub base: RapierShapeBase2D,
}

impl RapierCircleShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            radius: 0.0,
            handle: invalid_handle(),
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierCircleShape2D {
    fn get_type(&self) -> ShapeType {
        ShapeType::CIRCLE
    }

    fn get_moment_of_inertia(&self, p_mass: f32, p_scale: Vector2) -> f32 {
        let a = self.radius * p_scale.x;
        let b = self.radius * p_scale.y;
        p_mass * (a * a + b * b) / 4.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        shape_create_circle(self.radius)
    }

    fn set_data(&mut self, p_data: Variant) {
        self.radius = p_data.to();
        self.base.configure(Rect2::new(
            -Vector2::splat(self.radius),
            Vector2::splat(self.radius) * 2.0,
        ));
    }

    fn get_data(&self) -> Variant {
        self.radius.to_variant()
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

impl Drop for RapierCircleShape2D {
    fn drop(&mut self) {
        self.destroy_rapier_shape();
    }
}
