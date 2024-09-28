#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::insert_id_rid;
use crate::servers::rapier_physics_singleton::PhysicsIds;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
use crate::types::*;
pub struct RapierHeightMapShape3D {
    base: RapierShapeBase,
}
impl RapierHeightMapShape3D {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes, physics_ids: &mut PhysicsIds) {
        let shape = Self {
            base: RapierShapeBase::new(rid),
        };
        insert_id_rid(shape.base.get_id(), rid, physics_ids);
        physics_shapes.insert(rid, RapierShape::RapierHeightMapShape3D(shape));
    }
}
impl IRapierShape for RapierHeightMapShape3D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CONVEX_POLYGON
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        let width;
        let depth;
        let heights;
        if let Ok(dictionary) = data.try_to::<Dictionary>() {
            let in_width = dictionary.get_or_nil("width");
            let in_depth = dictionary.get_or_nil("depth");
            let new_heights = dictionary.get_or_nil("heights");
            if let Ok(in_width) = in_width.try_to::<i32>()
                && let Ok(in_depth) = in_depth.try_to::<i32>()
                && let Ok(in_heights) = new_heights.try_to::<PackedFloatArray>()
            {
                width = in_width;
                depth = in_depth;
                heights = in_heights;
                // else if let Ok(image) = heights.try_to::<Object>() {
                // TODO image support
                //}
                // Compute min and max heights or use precomputed values.
                let mut min_height: real = 0.0;
                let mut max_height: real = 0.0;
                if let Some(new_min_height) = dictionary.get("min_height")
                    && let Some(new_max_height) = dictionary.get("max_height")
                {
                    let new_min_height = variant_to_float(&new_min_height);
                    let new_max_height = variant_to_float(&new_max_height);
                    min_height = new_min_height;
                    max_height = new_max_height;
                } else {
                    let heights_size = heights.len();
                    for i in 0..heights_size {
                        let h = heights[i];
                        if h < min_height {
                            min_height = h;
                        } else if h > max_height {
                            max_height = h;
                        }
                    }
                }
                if min_height > max_height {
                    godot_error!("Invalid heightmap shape data");
                    return;
                }
            } else {
                godot_error!("Invalid heightmap dictionary data. Got {}", data);
                return;
            }
        } else {
            godot_error!("Invalid heightmap shape data type. Got {}", data);
            return;
        }
        if heights.len() != (width * depth) as usize {
            godot_error!("Invalid heightmap shape data");
            return;
        }
        if width <= 1 || depth <= 1 {
            godot_error!("Heightmap must have width and depth at least 2");
            return;
        }
        let handle = physics_engine.shape_create_heightmap(heights.as_slice(), width, depth);
        self.base.set_handle_and_reset_aabb(handle, physics_engine);
    }

    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let mut dictionary = Dictionary::new();
        let (heights, depth, width) = physics_engine.shape_get_heightmap(self.base.get_handle());
        let _ = dictionary.insert("width", width);
        let _ = dictionary.insert("depth", depth);
        let mut packed_heights = PackedFloatArray::default();
        for h in heights.iter() {
            packed_heights.push(*h)
        }
        let _ = dictionary.insert("heights", packed_heights);
        dictionary.to_variant()
    }
}
