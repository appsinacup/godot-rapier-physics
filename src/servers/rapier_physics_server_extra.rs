use godot::engine::utilities::rid_allocate_id;
use godot::engine::utilities::rid_from_int64;
use godot::prelude::*;

use super::rapier_physics_singleton::physics_data;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::fluids::rapier_fluid::RapierFluid;
use crate::servers::RapierPhysicsServer;
use crate::types::*;
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
    fn body_set_extra_param(body: Rid, param: i32, value: Variant) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_extra_param(
                    RapierBodyParam::from(param),
                    value,
                    &mut physics_data.physics_engine,
                );
            }
        }
    }

    #[func]
    fn body_get_extra_param(body: Rid, param: i32) -> Variant {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_extra_param(RapierBodyParam::from(param));
            }
        }
        0.0.to_variant()
    }

    #[cfg(feature = "serde-serialize")]
    #[func]
    fn joints_export_json() -> String {
        let physics_data = physics_data();
        let values = physics_data.joints.values().clone().collect::<Vec<_>>();
        match serde_json::to_string_pretty(&values) {
            Ok(s) => s,
            Err(err) => {
                godot_error!("{}", err);
                "{}".to_string()
            }
        }
    }

    #[cfg(feature = "serde-serialize")]
    #[func]
    fn joint_export_json(joint: Rid) -> String {
        let physics_data = physics_data();
        if let Some(joint) = physics_data.joints.get(&joint) {
            match serde_json::to_string_pretty(&joint) {
                Ok(s) => {
                    return s;
                }
                Err(err) => {
                    godot_error!("{}", err);
                    return "{}".to_string();
                }
            }
        }
        "{}".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    #[func]
    fn shapes_export_json() -> String {
        let physics_data = physics_data();
        let values = physics_data.shapes.values().clone().collect::<Vec<_>>();
        match serde_json::to_string_pretty(&values) {
            Ok(s) => s,
            Err(err) => {
                godot_error!("{}", err);
                "{}".to_string()
            }
        }
    }

    #[cfg(feature = "serde-serialize")]
    #[func]
    fn shape_export_json(shape: Rid) -> String {
        let physics_data = physics_data();
        if let Some(shape) = physics_data.shapes.get(&shape) {
            match serde_json::to_string_pretty(&shape) {
                Ok(s) => {
                    return s;
                }
                Err(err) => {
                    godot_error!("{}", err);
                    return "{}".to_string();
                }
            }
        }
        "{}".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    #[func]
    fn space_export_json(space: Rid) -> String {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space) {
            return space.export_json(&mut physics_data.physics_engine);
        }
        "{}".to_string()
    }

    #[func]
    fn collision_objects_export_json() -> String {
        let physics_data = physics_data();
        let _values = physics_data
            .collision_objects
            .values()
            .clone()
            .collect::<Vec<_>>();
        //match serde_json::to_string_pretty(&values) {
        //    Ok(s) => s,
        //    Err(err) => {
        //        godot_error!("{}", err);
        //        "{}".to_string()
        //    }
        //}
        "{}".to_string()
    }

    #[func]
    fn collision_object_export_json(_collision_object: Rid) -> String {
        let physics_data = physics_data();
        "{}".to_string()
    }

    #[func]
    fn fluid_create() -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let fluid = RapierFluid::new(rid);
        physics_data.fluids.insert(rid, fluid);
        rid
    }

    #[func]
    fn fluid_set_space(fluid_rid: Rid, space_rid: Rid) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_space(space_rid);
        }
    }

    #[func]
    fn fluid_set_density(fluid_rid: Rid, density: f64) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_density(density);
        }
    }

    #[func]
    fn fluid_set_effects(fluid_rid: Rid, params: Array<Gd<Resource>>) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_effects(params);
        }
    }

    #[func]
    fn fluid_get_points(fluid_rid: Rid) -> PackedVectorArray {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_points().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_get_velocities(fluid_rid: Rid) -> PackedVectorArray {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_velocities().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_get_accelerations(fluid_rid: Rid) -> PackedVectorArray {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_accelerations().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_set_points(fluid_rid: Rid, points: PackedVectorArray) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_points(points.to_vec());
        }
    }

    #[func]
    fn fluid_set_points_and_velocities(
        fluid_rid: Rid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_points_and_velocities(points.to_vec(), velocities.to_vec());
        }
    }

    #[func]
    fn fluid_add_points_and_velocities(
        fluid_rid: Rid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.add_points_and_velocities(points.to_vec(), velocities.to_vec());
        }
    }

    #[func]
    fn fluid_delete_points(fluid_rid: Rid, indices: PackedInt32Array) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.delete_points(indices.to_vec());
        }
    }
}
