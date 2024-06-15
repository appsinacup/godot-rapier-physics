use godot::classes::physics_server_2d::*;
use godot::prelude::*;
use rapier::math::{Real, Vector};

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::*;
pub struct RapierCapsuleShape2D {
    height: f32,
    radius: f32,
    pub base: RapierShapeBase,
}
impl RapierCapsuleShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        }
    }
}
impl IRapierShape for RapierCapsuleShape2D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CAPSULE
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector<Real>) -> f32 {
        let he2 = Vector::new(self.radius * 2.0, self.height).cross(&scale);
        mass * he2.dot(&he2) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        shape_create_capsule((self.height / 2.0) - self.radius, self.radius)
    }

    fn set_data(&mut self, data: Variant) {
        match data.get_type() {
            VariantType::ARRAY => {
                let arr: Array<f32> = data.to();
                if arr.len() != 2 {
                    return;
                }
                self.height = arr.at(0);
                self.radius = arr.at(1);
            }
            VariantType::VECTOR2 => {
                let vector_data: Vector2 = data.to();
                self.height = vector_data.y;
                self.radius = vector_data.x;
            }
            _ => {
                godot_error!("Invalid shape data");
                return;
            }
        }
        let base = self.base;
        if base.get_handle().is_valid() {
            base.destroy_rapier_shape();
        }
        base.configure(self.create_rapier_shape());
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
