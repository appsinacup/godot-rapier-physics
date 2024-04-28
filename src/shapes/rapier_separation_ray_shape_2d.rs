use crate::rapier2d::handle::{invalid_handle, Handle};
use crate::rapier2d::shape::shape_destroy;
use crate::shapes::rapier_shape_2d::{IRapierShape2D, RapierShapeBase2D};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierSeparationRayShape2D {
    length: f32,
    slide_on_slope: bool,
    handle: Handle,

    pub base: RapierShapeBase2D,
}

impl RapierSeparationRayShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            length: 0.0,
            slide_on_slope: false,
            handle: invalid_handle(),

            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierSeparationRayShape2D {
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
        return invalid_handle();
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() != VariantType::Dictionary {
            godot_error!("Invalid data type for WorldBoundaryShape2D.");
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
        return dictionary.to_variant();
    }

    fn get_rapier_shape(&mut self) -> Handle {
        self.handle
    }

    fn destroy_rapier_shape(&mut self) {
        if self.handle.is_valid() {
            shape_destroy(self.handle);
            self.handle = invalid_handle();
        }
    }
}

impl Drop for RapierSeparationRayShape2D {
    fn drop(&mut self) {
        self.destroy_rapier_shape();
    }
}
