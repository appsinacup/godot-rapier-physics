use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::*;
use crate::Vector;
#[derive(Serialize, Deserialize, Debug)]
pub struct RapierCylinderShape3D {
    height: f32,
    radius: f32,
    base: RapierShapeBase,
}
impl RapierCylinderShape3D {
    pub fn new(rid: Rid) -> Self {
        Self {
            height: 0.0,
            radius: 0.0,
            base: RapierShapeBase::new(rid),
        }
    }
}
impl IRapierShape for RapierCylinderShape3D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CYLINDER
    }

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

    fn create_rapier_shape(&mut self) -> Handle {
        shape_create_cylinder((self.height / 2.0) - self.radius, self.radius)
    }

    fn set_data(&mut self, data: Variant) {
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
        let handle = self.create_rapier_shape();
        self.base.set_handle(handle, self.compute_aabb());
    }

    fn get_data(&self) -> Variant {
        Vector2::new(self.radius, self.height).to_variant()
    }

    fn get_handle(&mut self) -> Handle {
        self.base.get_handle()
    }
}
impl RapierCylinderShape3D {
    fn compute_aabb(&self) -> Aabb {
        let he = Vector3::new(self.radius, self.height * 0.5, self.radius);
        Aabb::new(-he, he * 2.0)
    }
}
