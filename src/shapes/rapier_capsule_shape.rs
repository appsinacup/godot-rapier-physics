#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use serde::*;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::PhysicsData;
use crate::shapes::rapier_shape::*;
use crate::Vector;
//#[derive(Serialize, Deserialize, Debug)]
pub struct RapierCapsuleShape {
    height: f32,
    radius: f32,
    base: RapierShapeBase,
}
impl RapierCapsuleShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        }
    }
}
impl IRapierShape for RapierCapsuleShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CAPSULE
    }

    #[cfg(feature = "dim2")]
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> f32 {
        let he2 = Vector::new(self.radius * 2.0, self.height) * scale;
        mass * he2.dot(he2) / 12.0
    }

    #[cfg(feature = "dim3")]
    fn get_moment_of_inertia(&self, p_mass: f32, _scale: Vector) -> Vector3 {
        // use bad AABB approximation
        let extents = self.compute_aabb().size * 0.5;
        Vector3::new(
            (p_mass / 3.0) * (extents.y * extents.y + extents.z * extents.z),
            (p_mass / 3.0) * (extents.x * extents.x + extents.z * extents.z),
            (p_mass / 3.0) * (extents.x * extents.x + extents.y * extents.y),
        )
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        shape_create_capsule(
            (self.height / 2.0) - self.radius,
            self.radius,
            physics_engine,
        )
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        match data.get_type() {
            VariantType::ARRAY => {
                let arr: Array<f32> = data.to();
                if arr.len() != 2 {
                    return;
                }
                self.height = arr.at(0);
                self.radius = arr.at(1);
            }
            VariantType::VECTOR2 => {
                let vector_data: Vector2 = data.to();
                self.height = vector_data.y;
                self.radius = vector_data.x;
            }
            VariantType::DICTIONARY => {
                let dictionary: Dictionary = data.to();
                if let Some(height) = dictionary.get("height") {
                    if let Ok(height) = height.try_to::<real>() {
                        self.height = height;
                    }
                }
                if let Some(radius) = dictionary.get("radius") {
                    if let Ok(radius) = radius.try_to::<real>() {
                        self.radius = radius;
                    }
                }
            }
            _ => {
                godot_error!("Invalid shape data");
                return;
            }
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base
            .set_handle(handle, self.compute_aabb(), physics_engine);
    }

    fn get_data(&self) -> Variant {
        Vector2::new(self.radius, self.height).to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
impl RapierCapsuleShape {
    #[cfg(feature = "dim2")]
    fn compute_aabb(&self) -> Rect2 {
        let he = Vector2::new(self.radius, self.height * 0.5);
        Rect2::new(-he, he * 2.0)
    }

    #[cfg(feature = "dim3")]
    fn compute_aabb(&self) -> Aabb {
        let he = Vector3::new(self.radius, self.height * 0.5, self.radius);
        Aabb::new(-he, he * 2.0)
    }
}
