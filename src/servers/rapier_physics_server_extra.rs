pub enum RapierBodyParam {
    ContactSkin,
    Dominance,
}
impl RapierBodyParam {
    pub fn from_i32(value: i32) -> RapierBodyParam {
        match value {
            0 => RapierBodyParam::ContactSkin,
            1 => RapierBodyParam::Dominance,
            _ => RapierBodyParam::ContactSkin,
        }
    }
}
#[macro_export]
macro_rules! make_rapier_server_godot_impl {
    ($class: ident) => {
        use godot::global::rid_allocate_id;
        use godot::global::rid_from_int64;
        use $crate::bodies::rapier_collision_object::IRapierCollisionObject;
        use $crate::fluids::rapier_fluid::RapierFluid;
        use $crate::servers::RapierPhysicsServer;
        use $crate::servers::rapier_physics_server_extra::RapierBodyParam;
        #[godot_api]
        impl $class {
            #[func]
            /// Set an extra parameter for a body.
            /// If [param param] is [code]0[/code], sets the body's contact skin value.
            /// If [param param] is [code]1[/code], sets the body's dominance value.
            fn body_set_extra_param(body: Rid, param: i32, value: Variant) {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get_mut(&body) {
                    if let Some(body) = body.get_mut_body() {
                        body.set_extra_param(
                            RapierBodyParam::from_i32(param),
                            value,
                            &mut physics_data.physics_engine,
                        );
                    }
                }
            }

            #[func]
            /// Get an extra parameter for a body.
            /// If [param param] is [code]0[/code], gets the body's contact skin value.
            /// If [param param] is [code]1[/code], gets the body's dominance value.
            fn body_get_extra_param(body: Rid, param: i32) -> Variant {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body) {
                    if let Some(body) = body.get_body() {
                        return body.get_extra_param(RapierBodyParam::from_i32(param));
                    }
                }
                0.0.to_variant()
            }

            #[cfg(feature = "serde-serialize")]
            #[func]
            /// Exports the physics object to a JSON string. This is slower than the binary export.
            fn export_json(physics_object: Rid) -> String {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&physics_object) {
                    return body.export_json();
                }
                use $crate::shapes::rapier_shape::IRapierShape;
                if let Some(shape) = physics_data.shapes.get(&physics_object) {
                    return shape.get_base().export_json();
                }
                use $crate::joints::rapier_joint::IRapierJoint;
                if let Some(joint) = physics_data.joints.get(&physics_object) {
                    return joint.get_base().export_json();
                }
                if let Some(space) = physics_data.spaces.get(&physics_object) {
                    return space.export_json(&mut physics_data.physics_engine);
                }
                "".to_string()
            }

            #[cfg(feature = "serde-serialize")]
            #[func]
            /// Exports the physics object to a binary format.
            fn export_binary(physics_object: Rid) -> PackedByteArray {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&physics_object) {
                    return body.export_binary();
                }
                use $crate::shapes::rapier_shape::IRapierShape;
                if let Some(shape) = physics_data.shapes.get(&physics_object) {
                    return shape
                        .get_base()
                        .export_binary(&mut physics_data.physics_engine);
                }
                use $crate::joints::rapier_joint::IRapierJoint;
                if let Some(joint) = physics_data.joints.get(&physics_object) {
                    return joint.get_base().export_binary();
                }
                if let Some(space) = physics_data.spaces.get(&physics_object) {
                    return space.export_binary(&mut physics_data.physics_engine);
                }
                PackedByteArray::default()
            }

            #[cfg(feature = "serde-serialize")]
            #[func]
            /// Imports the physics object from a binary format.
            fn import_binary(physics_object: Rid, data: PackedByteArray) {
                use $crate::joints::rapier_joint::IRapierJoint;
                use $crate::servers::rapier_physics_singleton::insert_id_rid;
                use $crate::servers::rapier_physics_singleton::remove_id_rid;
                use $crate::shapes::rapier_shape::IRapierShape;
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get_mut(&physics_object) {
                    remove_id_rid(body.get_base().get_id(), &mut physics_data.ids);
                    body.import_binary(data);
                    insert_id_rid(
                        body.get_base().get_id(),
                        body.get_base().get_rid(),
                        &mut physics_data.ids,
                    );
                } else if let Some(shape) = physics_data.shapes.get_mut(&physics_object) {
                    remove_id_rid(shape.get_base().get_id(), &mut physics_data.ids);
                    // recreate shape handle
                    shape
                        .get_mut_base()
                        .destroy_shape(&mut physics_data.physics_engine);
                    shape
                        .get_mut_base()
                        .import_binary(data, &mut physics_data.physics_engine);
                    insert_id_rid(
                        shape.get_base().get_id(),
                        shape.get_base().get_rid(),
                        &mut physics_data.ids,
                    );
                } else if let Some(joint) = physics_data.joints.get_mut(&physics_object) {
                    remove_id_rid(joint.get_base().get_id(), &mut physics_data.ids);
                    joint.get_mut_base().import_binary(data);
                    insert_id_rid(
                        joint.get_base().get_id(),
                        joint.get_base().get_rid(),
                        &mut physics_data.ids,
                    );
                } else if let Some(space) = physics_data.spaces.get_mut(&physics_object) {
                    space.import_binary(&mut physics_data.physics_engine, data);
                }
            }

            #[func]
            /// Create a new fluid.
            pub(crate) fn fluid_create() -> Rid {
                let physics_data = physics_data();
                let rid = rid_from_int64(rid_allocate_id());
                let Ok(mut physics_singleton) =
                    PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
                else {
                    return Rid::Invalid;
                };
                let id = physics_singleton.bind_mut().implementation.next_id();
                let fluid = RapierFluid::new(id);
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
            /// Get interaction groups mask.
            pub(crate) fn fluid_get_collision_mask(fluid_rid: Rid) -> u32 {
                let physics_data = physics_data();
                if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
                    let interaction_groups = fluid.get_interaction_groups();
                    return interaction_groups.memberships.bits();
                }
                0
            }

            #[func]
            /// Set interaction groups mask.
            pub(crate) fn fluid_set_collision_mask(fluid_rid: Rid, mask: u32) {
                let physics_data = physics_data();
                if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
                    let mut interaction_groups = fluid.get_interaction_groups();
                    interaction_groups.memberships = mask.into();
                    fluid.set_interaction_groups(
                        interaction_groups,
                        &mut physics_data.physics_engine,
                    );
                }
            }

            #[func]
            /// Get interaction groups layer.
            pub(crate) fn fluid_get_collision_layer(fluid_rid: Rid) -> u32 {
                let physics_data = physics_data();
                if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
                    let interaction_groups = fluid.get_interaction_groups();
                    return interaction_groups.filter.bits();
                }
                0
            }

            pub(crate) fn fluid_set_collision_layer(fluid_rid: Rid, layer: u32) {
                let physics_data = physics_data();
                if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
                    let mut interaction_groups = fluid.get_interaction_groups();
                    interaction_groups.filter = layer.into();
                    fluid.set_interaction_groups(
                        interaction_groups,
                        &mut physics_data.physics_engine,
                    );
                }
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
                let Ok(physics_singleton) =
                    PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
                else {
                    return Array::default();
                };
                return physics_singleton
                    .bind()
                    .implementation
                    .space_get_active_bodies(space);
            }

            #[func]
            /// Get the bodies transform in the space.
            fn space_get_bodies_transform(space: Rid, bodies: Array<Rid>) -> Array<Transform> {
                let Ok(physics_singleton) =
                    PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
                else {
                    return Array::default();
                };
                return physics_singleton
                    .bind()
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
            /// Get the id of the object by rid. The id can be saved and used when reloading the scene.
            fn get_rapier_id(rid: Rid) -> i64 {
                let Ok(physics_singleton) =
                    PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
                else {
                    return 0;
                };
                return physics_singleton.bind().implementation.get_id(rid) as i64;
            }

            #[func]
            /// Get the global id of the physics server.
            fn get_global_id() -> i64 {
                let Ok(physics_singleton) =
                    PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
                else {
                    return 0;
                };
                return physics_singleton.bind().implementation.id as i64;
            }

            #[func]
            /// Set the global id of the physics server.
            fn set_global_id(id: i64) {
                let Ok(mut physics_singleton) =
                    PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
                else {
                    return;
                };
                physics_singleton.bind_mut().implementation.id = id as u64;
            }

            #[func]
            /// Get the stats of the physics server.
            fn get_stats() -> Dictionary {
                let mut dictionary = Dictionary::new();
                dictionary.set("ids", physics_data().ids.len() as i64);
                dictionary.set("active_spaces", physics_data().active_spaces.len() as i64);
                dictionary.set("spaces", physics_data().spaces.len() as i64);
                dictionary.set(
                    "collision_objects",
                    physics_data().collision_objects.len() as i64,
                );
                dictionary.set("fluids", physics_data().fluids.len() as i64);
                dictionary.set("joints", physics_data().joints.len() as i64);
                dictionary.set("shapes", physics_data().shapes.len() as i64);
                dictionary.set(
                    "physics_engine_shapes",
                    physics_data().physics_engine.shapes.len() as i64,
                );
                dictionary.set(
                    "physics_worlds",
                    physics_data().physics_engine.physics_worlds.len() as i64,
                );
                dictionary
            }
        }
    };
}
