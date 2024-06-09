use crate::shapes::rapier_shape::IRapierShape;
use crate::{
    rapier_wrapper::{handle::Handle, shape_2d::shape_create_circle},
    shapes::rapier_shape::RapierShapeBase,
};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierCircleShape2D {
    radius: f32,
    pub base: RapierShapeBase,
}

impl RapierCircleShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        }
    }
}

impl IRapierShape for RapierCircleShape2D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }
    fn get_type(&self) -> ShapeType {
        ShapeType::CIRCLE
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32 {
        let a = self.radius * scale.x;
        let b = self.radius * scale.y;
        mass * (a * a + b * b) / 4.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        shape_create_circle(self.radius)
    }

    fn set_data(&mut self, data: Variant) {
        match data.get_type() {
            VariantType::FLOAT | VariantType::INT => {
                self.radius = data.to();
            }
            _ => {
                godot_error!("Invalid shape data");
                return;
            }
        }
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
