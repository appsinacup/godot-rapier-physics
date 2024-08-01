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
pub struct RapierWorldBoundaryShape {
    normal: Vector,
    d: f32,

    base: RapierShapeBase,
}
impl RapierWorldBoundaryShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            normal: Vector::ZERO,
            d: 0.0,
            base: RapierShapeBase::new(rid),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierShape for RapierWorldBoundaryShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::WORLD_BOUNDARY
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        physics_engine.shape_create_halfspace(vector_to_rapier(self.normal), -self.d)
    }

    #[cfg(feature = "dim2")]
    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        if data.get_type() != VariantType::ARRAY {
            godot_error!("Invalid shape data");
            return;
        }
        let arr: Array<Variant> = data.try_to().unwrap_or_default();
        if arr.len() != 2 {
            godot_error!("Invalid data size for WorldBoundaryShape2D.");
            return;
        }
        if arr.at(0).get_type() != VariantType::VECTOR2
            || (arr.at(1).get_type() != VariantType::FLOAT
                && arr.at(1).get_type() != VariantType::INT)
        {
            godot_error!("Invalid data type for WorldBoundaryShape2D.");
            return;
        }
        self.normal = arr.at(0).try_to().unwrap_or_default();
        self.d = variant_to_float(&arr.at(1));
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle(handle, physics_engine);
    }

    #[cfg(feature = "dim3")]
    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        if data.get_type() != VariantType::PLANE {
            godot_error!("Invalid shape data");
            return;
        }
        let plane: Plane = data.try_to().unwrap_or(Plane::invalid());
        self.normal = plane.normal;
        self.d = plane.d;
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle(handle, physics_engine);
    }

    #[cfg(feature = "dim2")]
    fn get_data(&self) -> Variant {
        let mut arr = Array::<Variant>::new();
        arr.push(self.normal.to_variant());
        arr.push(self.d.to_variant());
        arr.to_variant()
    }

    #[cfg(feature = "dim3")]
    fn get_data(&self) -> Variant {
        let plane = Plane::new(self.normal, self.d);
        plane.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
