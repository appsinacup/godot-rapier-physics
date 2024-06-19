#[cfg(feature = "dim2")]
use godot::engine::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::engine::physics_server_3d::ShapeType;
use godot::prelude::*;
use serde::*;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::PhysicsData;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::Angle;
use crate::Vector;
//#[derive(Serialize, Deserialize, Debug)]
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

    #[cfg(feature = "dim2")]
    fn get_moment_of_inertia(&self, _mass: f32, _scale: Vector) -> Angle {
        f32::MAX
    }

    #[cfg(feature = "dim3")]
    fn get_moment_of_inertia(&self, _mass: f32, _scale: Vector) -> Angle {
        Vector3::new(f32::MAX, f32::MAX, f32::MAX)
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> Handle {
        shape_create_halfspace(vector_to_rapier(self.normal), -self.d, physics_engine)
    }

    #[cfg(feature = "dim2")]
    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
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
        let handle = self.create_rapier_shape(physics_engine);
        let rect = Rect2::new(Vector2::new(-1e4, -1e4), Vector2::new(1e4 * 2.0, 1e4 * 2.0));
        self.base.set_handle(handle, rect, physics_engine);
    }

    #[cfg(feature = "dim3")]
    fn set_data(&mut self, data: Variant) {
        if data.get_type() != VariantType::PLANE {
            godot_error!("Invalid shape data");
            return;
        }
        let plane: Plane = data.to();
        self.normal = plane.normal;
        self.d = plane.d;
        let handle = self.create_rapier_shape();
        let rect = Aabb::new(
            Vector::new(-1e4, -1e4, -1e4),
            Vector::new(1e4 * 2.0, 1e4 * 2.0, 1e4 * 2.0),
        );
        self.base.set_handle(handle, rect);
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

    fn get_handle(&self) -> Handle {
        self.base.get_handle()
    }
}
