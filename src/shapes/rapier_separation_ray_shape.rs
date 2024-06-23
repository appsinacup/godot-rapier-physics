#[cfg(feature = "dim2")]
use godot::engine::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::engine::physics_server_3d::ShapeType;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::types::*;
//#[derive(Serialize, Deserialize, Debug)]
pub struct RapierSeparationRayShape {
    length: f32,
    slide_on_slope: bool,
    base: RapierShapeBase,
}
impl RapierSeparationRayShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            length: 0.0,
            slide_on_slope: false,
            base: RapierShapeBase::new(rid),
        }
    }
}
impl IRapierShape for RapierSeparationRayShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::SEPARATION_RAY
    }

    #[cfg(feature = "dim2")]
    fn get_moment_of_inertia(&self, _mass: f32, _scale: Vector) -> f32 {
        0.0
    }

    #[cfg(feature = "dim3")]
    fn get_moment_of_inertia(&self, _mass: f32, _scale: Vector) -> Vector3 {
        Vector3::new(0.0, 0.0, 0.0)
    }

    fn allows_one_way_collision(&self) -> bool {
        false
    }

    fn create_rapier_shape(&mut self, _physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        ShapeHandle::default()
    }

    fn set_data(&mut self, data: Variant, _physics_engine: &mut PhysicsEngine) {
        if data.get_type() != VariantType::DICTIONARY {
            godot_error!("Invalid shape data.");
            return;
        }
        let dictionary: Dictionary = data.to();
        if !dictionary.contains_key("length") && !dictionary.contains_key("slide_on_slope") {
            godot_error!("Invalid shape data.");
            return;
        }
        self.length = dictionary.get_or_nil("length").to();
        self.slide_on_slope = dictionary.get_or_nil("slide_on_slope").to();
    }

    fn get_data(&self) -> Variant {
        let mut dictionary = Dictionary::new();
        dictionary.set("length", self.length);
        dictionary.set("slide_on_slope", self.slide_on_slope);
        dictionary.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
