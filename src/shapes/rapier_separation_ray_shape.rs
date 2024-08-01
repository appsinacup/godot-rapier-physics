#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::ShapeType;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
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
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
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
        let dictionary: Dictionary = data.try_to().unwrap_or_default();
        if !dictionary.contains_key("length") && !dictionary.contains_key("slide_on_slope") {
            godot_error!("Invalid shape data.");
            return;
        }
        self.length = dictionary.get_or_nil("length").try_to().unwrap_or_default();
        self.slide_on_slope = dictionary
            .get_or_nil("slide_on_slope")
            .try_to()
            .unwrap_or_default();
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
