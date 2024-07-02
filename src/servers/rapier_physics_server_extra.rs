use godot::engine::utilities::rid_allocate_id;
use godot::engine::utilities::rid_from_int64;
use godot::prelude::*;
use hashbrown::HashMap;

use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::fluids::rapier_fluid::RapierFluid;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::handle::WorldHandle;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::servers::RapierPhysicsServer;
use crate::shapes::rapier_shape::IRapierShape;
use crate::spaces::rapier_space::RapierSpace;
use crate::types::*;
pub enum RapierBodyParam {
    ContactSkin = 0,
}
pub type PhysicsShapes = HashMap<Rid, Box<dyn IRapierShape>>;
pub type PhysicsSpaces = HashMap<Rid, RapierSpace>;
pub type PhysicsActiveSpaces = HashMap<WorldHandle, Rid>;
pub type PhysicsCollisionObjects = HashMap<Rid, Box<dyn IRapierCollisionObject>>;
pub type PhysicsJoints = HashMap<Rid, Box<dyn IRapierJoint>>;
pub type PhysicsFluids = HashMap<Rid, RapierFluid>;
#[derive(Default)]
pub struct PhysicsData {
    pub shapes: PhysicsShapes,
    pub spaces: PhysicsSpaces,
    pub active_spaces: PhysicsActiveSpaces,
    pub collision_objects: PhysicsCollisionObjects,
    pub joints: PhysicsJoints,
    pub fluids: PhysicsFluids,
    pub physics_engine: PhysicsEngine,
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return 0.0.to_variant();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return "{}".to_string();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return "{}".to_string();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return "{}".to_string();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return "{}".to_string();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return "{}".to_string();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(space) = physics_data.spaces.get(&space) {
            return space.export_json(&mut physics_data.physics_engine);
        }
        "{}".to_string()
    }

    #[func]
    fn collision_objects_export_json() -> String {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return "{}".to_string();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return "{}".to_string();
        };
        let _physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        "{}".to_string()
    }

    #[func]
    fn fluid_create() -> Rid {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return Rid::Invalid;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        let rid = rid_from_int64(rid_allocate_id());
        let fluid = RapierFluid::new(rid);
        physics_data.fluids.insert(rid, fluid);
        rid
    }

    #[func]
    fn fluid_set_space(fluid_rid: Rid, space_rid: Rid) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_space(space_rid);
        }
    }

    #[func]
    fn fluid_set_density(fluid_rid: Rid, density: f64) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_density(density);
        }
    }

    #[func]
    fn fluid_set_effects(fluid_rid: Rid, params: Array<Gd<Resource>>) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.set_effects(params);
        }
    }

    #[func]
    fn fluid_get_points(fluid_rid: Rid) -> PackedVectorArray {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return PackedVectorArray::default();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_points().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_get_velocities(fluid_rid: Rid) -> PackedVectorArray {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return PackedVectorArray::default();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_velocities().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_get_accelerations(fluid_rid: Rid) -> PackedVectorArray {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return PackedVectorArray::default();
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get(&fluid_rid) {
            return PackedVectorArray::from(fluid.get_accelerations().as_slice());
        }
        PackedVectorArray::default()
    }

    #[func]
    fn fluid_set_points(fluid_rid: Rid, points: PackedVectorArray) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
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
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.add_points_and_velocities(points.to_vec(), velocities.to_vec());
        }
    }

    #[func]
    fn fluid_delete_points(fluid_rid: Rid, indices: PackedInt32Array) {
        let Ok(mut physics_singleton) =
            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
        else {
            return;
        };
        let physics_data = &mut physics_singleton.bind_mut().implementation.physics_data;
        if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
            fluid.delete_points(indices.to_vec());
        }
    }
}
