#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::ShapeType;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::types::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierRectangleShape {
    half_extents: Vector,
    base: RapierShapeBase,
}
impl RapierRectangleShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            half_extents: Vector::ZERO,
            base: RapierShapeBase::new(rid),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierShape for RapierRectangleShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    #[cfg(feature = "dim2")]
    fn get_type(&self) -> ShapeType {
        ShapeType::RECTANGLE
    }

    #[cfg(feature = "dim3")]
    fn get_type(&self) -> ShapeType {
        ShapeType::BOX
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        let v = vector_to_rapier(self.half_extents) * 2.0;
        physics_engine.shape_create_box(v)
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        if let Ok(v) = data.try_to() {
            self.half_extents = v;
            let handle = self.create_rapier_shape(physics_engine);
            self.base.set_handle(handle, physics_engine);
        } else {
            godot_error!("Invalid data type for RapierRectangleShape");
        }
    }

    fn get_data(&self) -> Variant {
        self.half_extents.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
