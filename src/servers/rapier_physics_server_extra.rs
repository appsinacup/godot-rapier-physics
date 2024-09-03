use godot::global::rid_allocate_id;
use godot::global::rid_from_int64;
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
    /// Set an extra parameter for a body.
    /// Right now only parameter available is 0, which is the contact skin.
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
    /// Get an extra parameter for a body.
    /// Right now only parameter available is 0, which is the contact skin.
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
    /// Exports the space to a JSON string. This is slower than the binary export.
    fn space_export_json(space: Rid) -> String {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space) {
            return space.export_space_json(&mut physics_data.physics_engine);
        }
        "".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    #[func]
    /// Exports the space to a binary format.
    fn space_export_binary(space: Rid) -> PackedByteArray {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space) {
            return space.export_space_binary(&mut physics_data.physics_engine);
        }
        PackedByteArray::default()
    }

    #[cfg(feature = "serde-serialize")]
    #[func]
    /// Imports the space from a binary format.
    fn space_import_binary(space: Rid, data: PackedByteArray) {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get_mut(&space) {
            space.import_space_binary(&mut physics_data.physics_engine, data);
        }
    }

    #[func]
    /// Create a new fluid.
    pub(crate) fn fluid_create() -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let fluid = RapierFluid::new();
        physics_data.fluids.insert(rid, fluid);
        rid
    }

    #[func]
    /// Set the space of the fluid.
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
    /// Set the density of the fluid.
    pub(crate) fn fluid_set_density(fluid_rid: Rid, density: real) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_density(density, &mut physics_data.physics_engine);
        }
    }

    #[func]
    /// Set the effects of the fluid.
    pub(crate) fn fluid_set_effects(fluid_rid: Rid, effects: Array<Option<Gd<Resource>>>) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_effects(effects, &mut physics_data.physics_engine);
        }
    }

    #[func]
    /// Get the points of the fluid particles.
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
    /// Get the velocities of the fluid particles.
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
    /// Get the accelerations of the fluid particles.
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
    /// Set the points of the fluid particles.
    pub(crate) fn fluid_set_points(fluid_rid: Rid, points: PackedVectorArray) {
        let physics_data = physics_data();
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_points(points.to_vec(), &mut physics_data.physics_engine);
        }
    }

    #[func]
    /// Set the velocities of the fluid particles.
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
    /// Add the points to the fluid particles.
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
    /// Delete the points of the fluid particles.
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
    /// Get the active bodies in the space.
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
    /// Get the bodies transform in the space.
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

    #[func]
    /// Step the space forward.
    fn space_step(space: Rid, delta: f32) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        physics_singleton
            .bind_mut()
            .implementation
            .space_step(&space, delta);
    }

    #[func]
    /// Flush the space queries. Used after space_step.
    fn space_flush_queries(space: Rid) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        physics_singleton
            .bind_mut()
            .implementation
            .space_flush_queries(&space);
    }

    #[func]
    /// Get the handle of the object by rid. The handle can be saved and used when reloading the scene.
    fn get_handle(rid: Rid) -> Array<i64> {
        let mut array = Array::new();
        array.resize(2, &0);
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return array;
        };
        let handle = physics_singleton.bind_mut().implementation.get_handle(rid);
        array.set(0, handle.0 as i64);
        array.set(1, handle.1 as i64);
        array
    }

    #[func]
    /// Set the handle of the object by rid.
    fn set_handle(rid: Rid, handle: i64) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        return physics_singleton
            .bind_mut()
            .implementation
            .set_handle(rid, handle);
    }
}
