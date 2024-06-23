#[cfg(feature = "dim2")]
use godot::engine::physics_server_2d::ShapeType;
#[cfg(feature = "dim3")]
use godot::engine::physics_server_3d::ShapeType;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::Rect;
use crate::Vector;
//#[derive(Serialize, Deserialize, Debug)]
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

    #[cfg(feature = "dim2")]
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> f32 {
        let he2 = self.half_extents * 2.0 * scale;
        mass * he2.dot(he2) / 12.0
    }

    #[cfg(feature = "dim3")]
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> Vector3 {
        let lx = self.half_extents.x;
        let ly = self.half_extents.y;
        let lz = self.half_extents.z;
        Vector3::new(
            (mass / 3.0) * (ly * ly + lz * lz),
            (mass / 3.0) * (lx * lx + lz * lz),
            (mass / 3.0) * (lx * lx + ly * ly),
        )
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
            let aabb = Rect::new(-self.half_extents, self.half_extents * 2.0);
            let handle = self.create_rapier_shape(physics_engine);
            self.base.set_handle(handle, aabb, physics_engine);
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
