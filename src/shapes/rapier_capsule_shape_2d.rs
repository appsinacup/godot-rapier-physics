use godot::classes::physics_server_2d::*;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::*;
use crate::Vector;
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

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> f32 {
        let he2 = Vector::new(self.radius * 2.0, self.height) * scale;
        mass * he2.dot(he2) / 12.0
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
        let he = Vector2::new(self.radius, self.height * 0.5);
        let rect = Rect2::new(-he, he * 2.0);
        let handle = self.create_rapier_shape();
        self.base.set_handle(handle, rect);
    }

    fn get_data(&self) -> Variant {
        Vector2::new(self.radius, self.height).to_variant()
    }

    fn get_handle(&mut self) -> Handle {
        self.base.get_handle()
    }
}
