#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::types::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierHeightMapShape3D {
    // TODO serialize this
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    heights: PackedFloatArray,
    width: i32,
    depth: i32,
    base: RapierShapeBase,
}
impl RapierHeightMapShape3D {
    pub fn new(rid: Rid) -> Self {
        Self {
            heights: PackedFloatArray::new(),
            width: 0,
            depth: 0,
            base: RapierShapeBase::new(rid),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
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

    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle {
        physics_engine.shape_create_heightmap(self.heights.as_slice(), self.width, self.depth)
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        match data.get_type() {
            VariantType::DICTIONARY => {
                if let Ok(dictionary) = data.try_to::<Dictionary>() {
                    let width = dictionary.get_or_nil("width");
                    let depth = dictionary.get_or_nil("depth");
                    let new_heights = dictionary.get_or_nil("heights");
                    if let Ok(width) = width.try_to::<i32>()
                        && let Ok(depth) = depth.try_to::<i32>()
                    {
                        if width <= 1 || depth <= 1 {
                            godot_error!("Heightmap must have width and depth at least 2");
                            return;
                        }
                        let heights: PackedFloatArray;
                        if let Ok(new_heights) = new_heights.try_to::<PackedFloatArray>() {
                            heights = new_heights;
                        }
                        // else if let Ok(image) = heights.try_to::<Object>() {
                        // TODO image support
                        //}
                        else {
                            godot_error!("Invalid heightmap shape data");
                            return;
                        }
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
                        self.heights = heights;
                        self.width = width;
                        self.depth = depth;
                    }
                }
            }
            _ => godot_error!("Invalid heightmap shape data"),
        }
        if self.heights.len() != (self.width * self.depth) as usize {
            godot_error!("Invalid heightmap shape data");
            return;
        }
        if self.width <= 1 || self.depth <= 1 {
            godot_error!("Heightmap must have width and depth at least 2");
            return;
        }
        let handle = self.create_rapier_shape(physics_engine);
        self.base.set_handle(handle, physics_engine);
    }

    fn get_data(&self) -> Variant {
        let mut dictionary = Dictionary::new();
        let _ = dictionary.insert("width", self.width);
        let _ = dictionary.insert("depth", self.depth);
        let _ = dictionary.insert("heights", self.heights.clone());
        dictionary.to_variant()
    }

    fn get_handle(&self) -> ShapeHandle {
        self.base.get_handle()
    }
}
