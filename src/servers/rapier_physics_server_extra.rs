use godot::prelude::*;
#[derive(GodotConvert, Var, Export, Debug, Clone, Copy, PartialEq)]
#[godot(via = i32)]
pub enum RapierBodyParam {
    ContactSkin,
    Dominance,
    SoftCcd,
}
impl RapierBodyParam {
    pub fn from_i32(value: i32) -> RapierBodyParam {
        match value {
            0 => RapierBodyParam::ContactSkin,
            1 => RapierBodyParam::Dominance,
            2 => RapierBodyParam::SoftCcd,
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
        use $crate::joints::rapier_joint::IRapierJoint;
        use $crate::joints::rapier_joint_base::RapierJointType;
        use $crate::servers::RapierPhysicsServer;
        use $crate::servers::rapier_physics_server_extra::RapierBodyParam;
        #[godot_api]
        impl $class {
            #[constant]
            pub const BODY_PARAM_CONTACT_SKIN: i32 = 0;
            #[constant]
            pub const BODY_PARAM_DOMINANCE: i32 = 1;
            #[constant]
            pub const BODY_PARAM_SOFT_CCD: i32 = 2;
            #[constant]
            pub const JOINT_TYPE: i32 = 0;
            #[constant]
            pub const JOINT_TYPE_INPULSE_JOINT: i32 = 0;
            #[constant]
            pub const JOINT_TYPE_MULTIBODY_JOINT: i32 = 1;
            #[constant]
            pub const JOINT_TYPE_MULTIBODY_KINEMATIC_JOINT: i32 = 2;

            #[func]
            /// Set an extra parameter for a body.
            /// If [param param] is [member BODY_PARAM_CONTACT_SKIN] (0), sets the body's contact skin value.
            /// If [param param] is [member BODY_PARAM_DOMINANCE] (1), sets the body's dominance value.
            /// If [param param] is [member BODY_PARAM_SOFT_CCD] (2), sets the body's soft_ccd value.
            pub fn body_set_extra_param(body: Rid, param: i32, value: Variant) {
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
            /// If [param param] is [member BODY_PARAM_CONTACT_SKIN] (0), gets the body's contact skin value.
            /// If [param param] is [member BODY_PARAM_DOMINANCE] (1), gets the body's dominance value.
            /// If [param param] is [member BODY_PARAM_SOFT_CCD] (2), gets the body's soft_ccd value.
            pub fn body_get_extra_param(body: Rid, param: i32) -> Variant {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body) {
                    if let Some(body) = body.get_body() {
                        return body.get_extra_param(RapierBodyParam::from_i32(param));
                    }
                }
                0.0.to_variant()
            }

            #[func]
            /// Set an extra parameter for a joint.
            /// If [param param] is [member JOINT_TYPE] (0), sets if multibody or not.
            /// Use [member JOINT_TYPE_INPULSE_JOINT] (0) for impulse joints, [member JOINT_TYPE_MULTIBODY_JOINT] (1) for multibody joints or [member JOINT_TYPE_MULTIBODY_KINEMATIC_JOINT] (2) for multibody kinematic joint.
            pub fn joint_set_extra_param(joint: Rid, param: i32, value: Variant) {
                if param == Self::JOINT_TYPE {
                    if let Ok(value) = value.try_to::<i32>() {
                        let joint_type = match value {
                            0 => RapierJointType::Impulse,
                            1 => RapierJointType::MultiBody,
                            2 => RapierJointType::MultiBodyKinematic,
                            _ => RapierJointType::Impulse, // default to Impulse
                        };
                        let Ok(mut physics_singleton) =
                            PhysicsServer::singleton().try_cast::<RapierPhysicsServer>()
                        else {
                            return;
                        };
                        physics_singleton
                            .bind_mut()
                            .implementation
                            .joint_change_type(joint, joint_type);
                    }
                }
            }

            #[func]
            /// Get an extra parameter for a joint.
            /// If [param param] is [member JOINT_TYPE] (0), gets if the joint is multibody or not.
            /// Returns [member JOINT_TYPE_INPULSE_JOINT] (0) for impulse joints, [member JOINT_TYPE_MULTIBODY_JOINT] (1) for multibody joints or [member JOINT_TYPE_MULTIBODY_KINEMATIC_JOINT] (2) for multibody kinematic joint.
            pub fn joint_get_extra_param(joint: Rid, param: i32) -> Variant {
                if param == Self::JOINT_TYPE {
                    let physics_data = physics_data();
                    if let Some(joint) = physics_data.joints.get(&joint) {
                        // Return 0 for Impulse, 1 for MultiBody
                        let joint_type = joint.get_base().get_joint_type();
                        return match joint_type {
                            RapierJointType::Impulse => 0.to_variant(),
                            RapierJointType::MultiBody => 1.to_variant(),
                            RapierJointType::MultiBodyKinematic => 2.to_variant(),
                        };
                    }
                }
                0.to_variant()
            }

            #[func]
            /// Solve inverse kinematics for a multibody joint to reach a target transform.
            /// Returns true if IK converged successfully.
            pub fn joint_solve_inverse_kinematics(joint: Rid, target_transform: Transform) -> bool {
                use $crate::rapier_wrapper::convert::vector_to_rapier;
                let physics_data = physics_data();
                if let Some(joint_obj) = physics_data.joints.get_mut(&joint) {
                    let space_handle = joint_obj.get_base().get_space_id();
                    let joint_handle = joint_obj.get_base().get_handle();
                    let custom_ik_options = joint_obj.get_base().custom_ik_options;
                    // Convert Transform to Isometry
                    let translation = vector_to_rapier(target_transform.origin);
                    #[cfg(feature = "dim2")]
                    let target_isometry =
                        rapier::prelude::Isometry::new(translation, target_transform.rotation());
                    #[cfg(feature = "dim3")]
                    let target_isometry = {
                        let quat = target_transform.basis.to_quat();
                        let rotation = rapier::prelude::Rotation::from_quaternion(
                            rapier::prelude::Quaternion::new(quat.w, quat.x, quat.y, quat.z),
                        );
                        rapier::prelude::Isometry::from_parts(translation.into(), rotation)
                    };
                    physics_data.physics_engine.multibody_solve_ik(
                        space_handle,
                        joint_handle,
                        target_isometry,
                        custom_ik_options,
                    )
                } else {
                    false
                }
            }

            #[func]
            /// Get the current end effector position/rotation for a multibody joint.
            pub fn joint_get_link_transform(joint: Rid) -> Transform {
                let physics_data = physics_data();
                if let Some(joint_obj) = physics_data.joints.get(&joint) {
                    let space_handle = joint_obj.get_base().get_space_id();
                    let joint_handle = joint_obj.get_base().get_handle();
                    if let Some(transform) = physics_data
                        .physics_engine
                        .multibody_get_link_transform(space_handle, joint_handle)
                    {
                        #[cfg(feature = "dim2")]
                        {
                            let angle = transform.rotation.angle();
                            let origin =
                                Vector2::new(transform.translation.x, transform.translation.y);
                            return Transform2D {
                                a: Vector2::new(angle.cos(), angle.sin()),
                                b: Vector2::new(-angle.sin(), angle.cos()),
                                origin,
                            };
                        }
                        #[cfg(feature = "dim3")]
                        {
                            let quat = transform.rotation.quaternion();
                            let basis =
                                Basis::from_quat(Quaternion::new(quat.w, quat.x, quat.y, quat.z));
                            let origin = Vector3::new(
                                transform.translation.x,
                                transform.translation.y,
                                transform.translation.z,
                            );
                            return Transform3D { basis, origin };
                        }
                    }
                }
                Transform::IDENTITY
            }

            #[func]
            /// Set custom IK options for a specific joint.
            /// This overrides the default Rapier IK parameters.
            /// constrained_axes: bitmask for which axes to constrain (1=X/Lin, 2=Y/Lin, 4=Z/Lin, 8=AngX, 16=AngY, 32=AngZ)
            ///   Common values: 3=XY position (2D), 7=XYZ position (3D), 56=rotation (3D), 63=all (3D)
            /// Default values: damping=1.0, max_iterations=8, constrained_axes=63, epsilon_linear=1e-6, epsilon_angular=1e-6
            pub fn joint_set_ik_options(
                joint: Rid,
                damping: real,
                max_iterations: i32,
                constrained_axes: i32,
                epsilon_linear: real,
                epsilon_angular: real,
            ) {
                let physics_data = physics_data();
                if let Some(joint_obj) = physics_data.joints.get_mut(&joint) {
                    use rapier::dynamics::InverseKinematicsOption;
                    use rapier::dynamics::JointAxesMask;
                    let options = InverseKinematicsOption {
                        damping,
                        max_iters: max_iterations as usize,
                        constrained_axes: JointAxesMask::from_bits_truncate(constrained_axes as u8),
                        epsilon_linear,
                        epsilon_angular,
                    };
                    joint_obj.get_mut_base().custom_ik_options = options;
                }
            }

            #[func]
            /// Reset IK options to Rapier's default values.
            /// Default values: damping=1.0, max_iterations=8, constrained_axes=63, epsilon_linear=1e-6, epsilon_angular=1e-6
            pub fn joint_reset_ik_options(joint: Rid) {
                let physics_data = physics_data();
                if let Some(joint_obj) = physics_data.joints.get_mut(&joint) {
                    use rapier::dynamics::InverseKinematicsOption;
                    joint_obj.get_mut_base().custom_ik_options = InverseKinematicsOption::default();
                }
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
            /// Get the indices of the fluid particles inside an AABB.
            pub(crate) fn fluid_get_particles_in_aabb(
                fluid_rid: Rid,
                aabb: $crate::types::Rect,
            ) -> PackedInt32Array {
                let physics_data = physics_data();
                if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
                    let indices =
                        fluid.get_particles_in_aabb(aabb, &mut physics_data.physics_engine);
                    return PackedInt32Array::from_iter(indices);
                }
                PackedInt32Array::default()
            }

            #[func]
            /// Get the indices of the fluid particles inside a ball.
            pub(crate) fn fluid_get_particles_in_ball(
                fluid_rid: Rid,
                center: $crate::types::Vector,
                radius: real,
            ) -> PackedInt32Array {
                let physics_data = physics_data();
                if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
                    let indices = fluid.get_particles_in_ball(
                        center,
                        radius,
                        &mut physics_data.physics_engine,
                    );
                    return PackedInt32Array::from_iter(indices);
                }
                PackedInt32Array::default()
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
            pub(crate) fn fluid_set_collision_masks(fluid_rid: Rid, mask: u32, layer: u32) {
                let physics_data = physics_data();
                if let Some(fluid) = physics_data.fluids.get_mut(&fluid_rid) {
                    let mut interaction_groups = fluid.get_interaction_groups();
                    interaction_groups.memberships = mask.into();
                    interaction_groups.filter = layer.into();
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
                RapierPhysicsServerImpl::space_flush_queries(&space);
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
