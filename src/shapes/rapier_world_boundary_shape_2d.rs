use crate::rapier2d::handle::Handle;
use crate::rapier2d::shape::shape_create_halfspace;
use crate::rapier2d::vector::Vector;
use crate::shapes::rapier_shape_2d::{IRapierShape2D, RapierShapeBase2D};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};

pub struct RapierWorldBoundaryShape2D {
    normal: Vector2,
    d: f32,

    pub base: RapierShapeBase2D,
}

impl RapierWorldBoundaryShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            normal: Vector2::ZERO,
            d: 0.0,
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierWorldBoundaryShape2D {
    fn get_base(&self) -> &RapierShapeBase2D {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierShapeBase2D {
        &mut self.base
    }
    fn get_type(&self) -> ShapeType {
        ShapeType::WORLD_BOUNDARY
    }

    fn get_moment_of_inertia(&self, _mass: f32, _scale: Vector2) -> f32 {
        0.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        let v = Vector {
            x: self.normal.x,
            y: -self.normal.y,
        };
        return shape_create_halfspace(&v, -self.d);
    }

    fn set_data(&mut self, data: Variant) {
        if data.get_type() != VariantType::Array {
            godot_error!("Invalid shape data");
            return;
        }
        let arr: Array<Variant> = data.to();
        if arr.len() != 2 {
            godot_error!("Invalid data size for WorldBoundaryShape2D.");
            return;
        }
        if arr.get(0).get_type() != VariantType::Vector2
            || arr.get(1).get_type() != VariantType::Float
        {
            godot_error!("Invalid data type for WorldBoundaryShape2D.");
            return;
        }
        self.normal = arr.get(0).to();
        self.d = arr.get(1).to();
        self.base.configure(Rect2::new(
            Vector2::new(-1e4, -1e4),
            Vector2::new(1e4 * 2.0, 1e4 * 2.0),
        ));
    }

    fn get_data(&self) -> Variant {
        let mut arr = Array::<Variant>::new();
        arr.push(self.normal.to_variant());
        arr.push(self.d.to_variant());
        return arr.to_variant();
    }

    fn get_rapier_shape(&mut self) -> Handle {
        if !self.base.get_handle().is_valid() {
            let handle = self.create_rapier_shape();
            self.base.set_handle(handle);
        }
        self.base.get_handle()
    }
}
