use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::{
    rapier2d::{handle::Handle, shape::shape_create_capsule},
    shapes::rapier_shape_2d::RapierShapeBase2D,
};
use godot::engine::physics_server_2d::ShapeType;
use godot::prelude::*;

pub struct RapierCapsuleShape2D {
    height: f32,
    radius: f32,
    pub base: RapierShapeBase2D,
}

impl RapierCapsuleShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierCapsuleShape2D {
    fn get_base(&self) -> &RapierShapeBase2D {
        &self.base
    }
    fn get_type(&self) -> ShapeType {
        ShapeType::CAPSULE
    }

    fn get_moment_of_inertia(&self, p_mass: f32, p_scale: Vector2) -> f32 {
        let he2 = Vector2::new(self.radius * 2.0, self.height) * p_scale;
        p_mass * he2.dot(he2) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        shape_create_capsule((self.height / 2.0) - self.radius, self.radius)
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() == VariantType::Array {
            let arr: Array<f32> = data.to();
            if arr.len() != 2 {
                godot_error!("Failed to convert Variant to Vector2");
                return;
            }
            self.height = arr.get(0);
            self.radius = arr.get(1);
        } else {
            let vector_data: Vector2 = data.to();
            self.height = vector_data.y;
            self.radius = vector_data.x;
        }

        let he = Vector2::new(self.radius, self.height * 0.5);
        self.base.configure(Rect2::new(-he, he * 2.0));
    }

    fn get_data(&self) -> Variant {
        Vector2::new(self.radius, self.height).to_variant()
    }
    fn get_rapier_shape(&mut self) -> Handle {
        if !self.base.get_handle().is_valid() {
            let handle = self.create_rapier_shape();
            self.base.set_handle(handle);
        }
        self.base.get_handle()
    }
}
