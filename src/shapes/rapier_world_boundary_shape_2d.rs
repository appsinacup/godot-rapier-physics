use godot::engine::physics_server_2d::ShapeType;
use godot::prelude::*;
use rapier::math::Real;
use rapier::math::Vector;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
pub struct RapierWorldBoundaryShape2D {
    normal: Vector2,
    d: f32,

    pub base: RapierShapeBase,
}
impl RapierWorldBoundaryShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            normal: Vector2::ZERO,
            d: 0.0,
            base: RapierShapeBase::new(rid),
        }
    }
}
impl IRapierShape for RapierWorldBoundaryShape2D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::WORLD_BOUNDARY
    }

    fn get_moment_of_inertia(&self, _mass: f32, _scale: Vector<Real>) -> f32 {
        f32::MAX
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        let v = rapier::math::Vector::<Real>::new(self.normal.x, -self.normal.y);
        shape_create_halfspace(v, -self.d)
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() != VariantType::ARRAY {
            godot_error!("Invalid shape data");
            return;
        }
        let arr: Array<Variant> = data.to();
        if arr.len() != 2 {
            godot_error!("Invalid data size for WorldBoundaryShape2D.");
            return;
        }
        if arr.at(0).get_type() != VariantType::VECTOR2
            || arr.at(1).get_type() != VariantType::FLOAT
        {
            godot_error!("Invalid data type for WorldBoundaryShape2D.");
            return;
        }
        self.normal = arr.at(0).to();
        self.d = arr.at(1).to();
        let handle = self.create_rapier_shape();
        self.base.set_handle(handle);
    }

    fn get_data(&self) -> Variant {
        let mut arr = Array::<Variant>::new();
        arr.push(self.normal.to_variant());
        arr.push(self.d.to_variant());
        arr.to_variant()
    }

    fn get_handle(&mut self) -> Handle {
        self.base.get_handle()
    }
}
