#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::ShapeType;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::insert_id_rid;
use crate::servers::rapier_physics_singleton::PhysicsIds;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
pub struct RapierSeparationRayShape {
    length: f32,
    slide_on_slope: bool,
    base: RapierShapeBase,
}
impl RapierSeparationRayShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes, physics_ids: &mut PhysicsIds) {
        let shape = Self {
            length: 0.0,
            slide_on_slope: false,
            base: RapierShapeBase::new(rid),
        };
        insert_id_rid(shape.base.get_id(), rid, physics_ids);
        physics_shapes.insert(rid, RapierShape::RapierSeparationRayShape(shape));
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

    fn allows_one_way_collision(&self) -> bool {
        false
    }

    fn set_data(&mut self, data: Variant, _physics_engine: &mut PhysicsEngine) {
        if data.get_type() != VariantType::DICTIONARY {
            godot_error!(
                "RapierSeparationRayShape data must be a dictionary. Got {}",
                data
            );
            return;
        }
        let dictionary: Dictionary = data.try_to().unwrap_or_default();
        if !dictionary.contains_key("length") && !dictionary.contains_key("slide_on_slope") {
            godot_error!("RapierSeparationRayShape data must contain 'length' and 'slide_on_slope' keys. Got {}", data);
            return;
        }
        self.length = dictionary.get_or_nil("length").try_to().unwrap_or_default();
        self.slide_on_slope = dictionary
            .get_or_nil("slide_on_slope")
            .try_to()
            .unwrap_or_default();
    }

    fn get_data(&self, _physics_engine: &PhysicsEngine) -> Variant {
        let mut dictionary = Dictionary::new();
        dictionary.set("length", self.length);
        dictionary.set("slide_on_slope", self.slide_on_slope);
        dictionary.to_variant()
    }
}
