use godot::global::rid_allocate_id;
use godot::global::rid_from_int64;
use godot::prelude::*;

use super::rapier_physics_singleton::physics_data;
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

    #[cfg(feature = "serde-serialize")]
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

    #[cfg(feature = "serde-serialize")]
    #[func]
    fn collision_object_export_json(_collision_object: Rid) -> String {
        "{}".to_string()
    }

    #[func]
    pub(crate) fn fluid_create() -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let fluid = RapierFluid::new(rid);
        physics_data.fluids.insert(rid, fluid);
        rid
    }

    #[func]
    pub(crate) fn fluid_set_space(fluid_rid: Rid, space_rid: Rid) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_space(
                space_rid,
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
            );
        }
    }

    #[func]
    pub(crate) fn fluid_set_density(fluid_rid: Rid, density: real) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_density(density, &mut physics_data.physics_engine);
        }
    }

    #[func]
    pub(crate) fn fluid_set_effects(fluid_rid: Rid, effects: Array<Option<Gd<Resource>>>) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_effects(effects, &mut physics_data.physics_engine);
        }
    }

    #[func]
    pub(crate) fn fluid_get_points(fluid_rid: Rid) -> PackedVectorArray {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            return PackedVectorArray::from(
                fluid
                    .get_points(&mut physics_data.physics_engine)
                    .as_slice(),
            );
        }
        PackedVectorArray::default()
    }

    #[func]
    pub(crate) fn fluid_get_velocities(fluid_rid: Rid) -> PackedVectorArray {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            return PackedVectorArray::from(
                fluid
                    .get_velocities(&mut physics_data.physics_engine)
                    .as_slice(),
            );
        }
        PackedVectorArray::default()
    }

    #[func]
    pub(crate) fn fluid_get_accelerations(fluid_rid: Rid) -> PackedVectorArray {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            return PackedVectorArray::from(
                fluid
                    .get_accelerations(&mut physics_data.physics_engine)
                    .as_slice(),
            );
        }
        PackedVectorArray::default()
    }

    #[func]
    pub(crate) fn fluid_set_points(fluid_rid: Rid, points: PackedVectorArray) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_points(points.to_vec(), &mut physics_data.physics_engine);
        }
    }

    #[func]
    pub(crate) fn fluid_set_points_and_velocities(
        fluid_rid: Rid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_points_and_velocities(
                points.to_vec(),
                velocities.to_vec(),
                &mut physics_data.physics_engine,
            );
        }
    }

    #[func]
    pub(crate) fn fluid_add_points_and_velocities(
        fluid_rid: Rid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.add_points_and_velocities(
                points.to_vec(),
                velocities.to_vec(),
                &mut physics_data.physics_engine,
            );
        }
    }

    #[func]
    pub(crate) fn fluid_delete_points(fluid_rid: Rid, indices: PackedInt32Array) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            let mut indices = indices.to_vec();
            indices.sort_unstable();
            indices.reverse();
            fluid.delete_points(indices, &mut physics_data.physics_engine);
        }
    }

    #[func]
    fn body_set_state_sync_callback(body: Rid, callable: Callable) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        physics_singleton
            .bind_mut()
            .implementation
            .body_set_state_sync_callback(body, callable);
    }

    #[func]
    fn space_get_active_bodies(space: Rid) -> Array<Rid> {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return Array::default();
        };
        return physics_singleton
            .bind_mut()
            .implementation
            .space_get_active_bodies(space);
    }

    #[func]
    fn space_get_bodies_transform(space: Rid, bodies: Array<Rid>) -> Array<Transform> {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return Array::default();
        };
        return physics_singleton
            .bind_mut()
            .implementation
            .space_get_bodies_transform(space, bodies);
    }
}
