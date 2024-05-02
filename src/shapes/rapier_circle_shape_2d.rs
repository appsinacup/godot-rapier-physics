use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::{
    rapier2d::{handle::Handle, shape::shape_create_circle},
    shapes::rapier_shape_2d::RapierShapeBase2D,
};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierCircleShape2D {
    radius: f32,
    pub base: RapierShapeBase2D,
}

impl RapierCircleShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            radius: 0.0,
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierCircleShape2D {
    fn get_base(&self) -> &RapierShapeBase2D {
        &self.base
    }
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
        if !self.base.get_handle().is_valid() {
            let handle = self.create_rapier_shape();
            self.base.set_handle(handle);
        }
        self.base.get_handle()
    }
}