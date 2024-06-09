use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::fluids::fluid_effect::FluidEffect;
use crate::fluids::rapier_fluid::RapierFluid;
use crate::servers::RapierPhysicsServer;
use crate::PackedVectorArray;
use godot::classes::IPhysicsServer2DExtension;
use godot::engine::utilities::{rid_allocate_id, rid_from_int64};
use godot::prelude::*;

use super::rapier_physics_singleton::{bodies_singleton, fluids_singleton, spaces_singleton};

pub enum RapierBodyParam {
    ContactSkin = 0,
}

impl From<i32> for RapierBodyParam {
    fn from(i: i32) -> Self {
        match i {
            0 => RapierBodyParam::ContactSkin,
            _ => RapierBodyParam::ContactSkin,
        }
    }
}

#[godot_api]
impl RapierPhysicsServer {
    #[func]
    fn body_set_extra_param(&mut self, body: Rid, param: i32, value: Variant) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_extra_param(RapierBodyParam::from(param), value);
            }
        }
    }
    #[func]
    fn body_get_extra_param(&self, body: Rid, param: i32) -> Variant {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_extra_param(RapierBodyParam::from(param));
            }
        }
        0.0.to_variant()
    }
    #[func]
    fn space_export_json(&mut self, world: Rid) -> String {
        if let Some(world) = spaces_singleton().spaces.get_mut(&world) {
            world.export_json();
        }
        "{}".to_string()
    }
    #[func]
    fn space_export_binary(&mut self, world: Rid) -> PackedByteArray {
        if let Some(world) = spaces_singleton().spaces.get_mut(&world) {
            world.export_binary();
        }
        PackedByteArray::new()
    }
    #[func]
    fn fluid_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let fluid = RapierFluid::new(rid);
        fluids_singleton().fluids.insert(rid, Box::new(fluid));
        rid
    }

    #[func]
    fn fluid_set_space(&mut self, fluid_rid: Rid, space_rid: Rid) {
        if let Some(fluid) = fluids_singleton().fluids.get_mut(&fluid_rid) {
            fluid.set_space(space_rid);
        }
    }

    #[func]
    fn fluid_set_density(&mut self, fluid_rid: Rid, density: f64) {
        if let Some(fluid) = fluids_singleton().fluids.get_mut(&fluid_rid) {
            fluid.set_density(density);
        }
    }

    #[func]
    fn fluid_set_effects(&mut self, fluid_rid: Rid, params: Array<Gd<FluidEffect>>) {
        if let Some(fluid) = fluids_singleton().fluids.get_mut(&fluid_rid) {
            fluid.set_effects(params);
        }
    }

    #[func]
    fn fluid_get_points(&self, fluid_rid: Rid) -> PackedVectorArray {
        if let Some(fluid) = fluids_singleton().fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_points().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_get_velocities(&self, fluid_rid: Rid) -> PackedVectorArray {
        if let Some(fluid) = fluids_singleton().fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_velocities().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_get_accelerations(&self, fluid_rid: Rid) -> PackedVectorArray {
        if let Some(fluid) = fluids_singleton().fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_accelerations().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_set_points(&mut self, fluid_rid: Rid, points: PackedVectorArray) {
        if let Some(fluid) = fluids_singleton().fluids.get_mut(&fluid_rid) {
            fluid.set_points(points.to_vec());
        }
    }

    #[func]
    fn fluid_set_points_and_velocities(
        &mut self,
        fluid_rid: Rid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        if let Some(fluid) = fluids_singleton().fluids.get_mut(&fluid_rid) {
            fluid.set_points_and_velocities(points.to_vec(), velocities.to_vec());
        }
    }

    #[func]
    fn fluid_add_points_and_velocities(
        &mut self,
        fluid_rid: Rid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        if let Some(fluid) = fluids_singleton().fluids.get_mut(&fluid_rid) {
            fluid.add_points_and_velocities(points.to_vec(), velocities.to_vec());
        }
    }

    #[func]
    fn fluid_delete_points(&mut self, fluid_rid: Rid, indices: PackedInt32Array) {
        if let Some(fluid) = fluids_singleton().fluids.get_mut(&fluid_rid) {
            fluid.delete_points(indices.to_vec());
        }
    }
}
