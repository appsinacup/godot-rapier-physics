#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::ShapeType;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
pub struct RapierRectangleShape {
    base: RapierShapeBase,
}
impl RapierRectangleShape {
    pub fn create(id: RapierId, rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            base: RapierShapeBase::new(id, rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierRectangleShape(shape));
    }
}
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

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        if let Ok(v) = data.try_to() {
            let half_extents = v;
            let v = vector_to_rapier(half_extents) * 2.0;
            physics_engine.shape_create_box(v, self.base.get_id());
            self.base.reset_aabb(physics_engine);
        } else {
            godot_error!("Invalid data type for RapierRectangleShape");
        }
    }

    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let half_extents = physics_engine.shape_get_box_size(self.base.get_id());
        vector_to_godot(half_extents).to_variant()
    }
}
