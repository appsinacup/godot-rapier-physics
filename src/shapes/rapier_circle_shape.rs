#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::*;
use crate::types::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierCircleShape {
    radius: real,
    base: RapierShapeBase,
}
impl RapierCircleShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierShape for RapierCircleShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    #[cfg(feature = "dim2")]
    fn get_type(&self) -> ShapeType {
        ShapeType::CIRCLE
    }

    #[cfg(feature = "dim3")]
    fn get_type(&self) -> ShapeType {
        ShapeType::SPHERE
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        physics_engine.shape_create_circle(self.radius)
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        match data.get_type() {
            VariantType::FLOAT | VariantType::INT => {
                self.radius = variant_to_float(&data);
            }
            _ => {
                godot_error!("Invalid shape data");
                return;
            }
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle(handle, physics_engine);
    }

    fn get_data(&self) -> Variant {
        self.radius.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
