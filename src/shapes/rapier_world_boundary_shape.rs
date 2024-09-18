#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::ShapeType;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
pub struct RapierWorldBoundaryShape {
    base: RapierShapeBase,
}
impl RapierWorldBoundaryShape {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierWorldBoundaryShape(shape));
    }
}
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

    #[cfg(feature = "dim2")]
    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        use crate::types::variant_to_float;
        if data.get_type() != VariantType::ARRAY {
            godot_error!("RapierWorldBoundaryShape data must be an array.");
            return;
        }
        let arr: Array<Variant> = data.try_to().unwrap_or_default();
        if arr.len() != 2 {
            godot_error!("RapierWorldBoundaryShape data must be an array of 2 elements.");
            return;
        }
        if arr.at(0).get_type() != VariantType::VECTOR2
            || (arr.at(1).get_type() != VariantType::FLOAT
                && arr.at(1).get_type() != VariantType::INT)
        {
            godot_error!(
                "RapierWorldBoundaryShape data must be an array of 2 elements. Got {}",
                data
            );
            return;
        }
        let normal = arr.at(0).try_to().unwrap_or_default();
        let d = variant_to_float(&arr.at(1));
        let handle = physics_engine.shape_create_halfspace(vector_to_rapier(normal), d);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    #[cfg(feature = "dim3")]
    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        if data.get_type() != VariantType::PLANE {
            godot_error!("RapierWorldBoundaryShape data must be a plane.");
            return;
        }
        let plane: Plane = data.try_to().unwrap_or(Plane::invalid());
        let normal = plane.normal;
        let d = plane.d;
        let handle = physics_engine.shape_create_halfspace(vector_to_rapier(normal), d);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    #[cfg(feature = "dim2")]
    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let (normal, d) = physics_engine.shape_get_halfspace(self.base.get_handle());
        let mut arr = Array::<Variant>::new();
        arr.push(vector_to_godot(normal).to_variant());
        arr.push(d.to_variant());
        arr.to_variant()
    }

    #[cfg(feature = "dim3")]
    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let (normal, d) = physics_engine.shape_get_halfspace(self.base.get_handle());
        let plane = Plane::new(vector_to_godot(normal), d);
        plane.to_variant()
    }
}
