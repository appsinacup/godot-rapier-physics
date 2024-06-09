use crate::rapier_wrapper::handle::{invalid_handle, Handle};
use crate::shapes::rapier_shape::{IRapierShape, RapierShapeBase};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierSeparationRayShape2D {
    length: f32,
    slide_on_slope: bool,
    pub base: RapierShapeBase,
}

impl RapierSeparationRayShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            length: 0.0,
            slide_on_slope: false,
            base: RapierShapeBase::new(rid),
        }
    }
}

impl IRapierShape for RapierSeparationRayShape2D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }
    fn get_type(&self) -> ShapeType {
        ShapeType::SEPARATION_RAY
    }

    fn get_moment_of_inertia(&self, _mass: f32, _scale: Vector2) -> f32 {
        0.0
    }

    fn allows_one_way_collision(&self) -> bool {
        false
    }

    fn create_rapier_shape(&mut self) -> Handle {
        invalid_handle()
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() != VariantType::DICTIONARY {
            godot_error!("Invalid shape data.");
            return;
        }
        let dictionary: Dictionary = data.to();
        self.length = dictionary.get_or_nil("length").to();
        self.slide_on_slope = dictionary.get_or_nil("slide_on_slope").to();
    }

    fn get_data(&self) -> Variant {
        let mut dictionary = Dictionary::new();
        dictionary.set("length", self.length);
        dictionary.set("slide_on_slope", self.slide_on_slope);
        dictionary.to_variant()
    }

    fn get_rapier_shape(&mut self) -> Handle {
        if !self.base.get_handle().is_valid() {
            let handle = self.create_rapier_shape();
            self.base.set_handle(handle);
        }
        self.base.get_handle()
    }
}
