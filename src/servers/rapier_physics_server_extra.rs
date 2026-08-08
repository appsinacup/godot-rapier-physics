use godot::prelude::*;
#[derive(GodotConvert, Var, Export, Debug, Clone, Copy, PartialEq)]
#[godot(via = i32)]
pub enum RapierBodyParam {
    ContactSkin,
    Dominance,
    SoftCcd,
    Massless,
    AdditionalSolverIterations,
}
impl RapierBodyParam {
    pub fn from_i32(value: i32) -> RapierBodyParam {
        match value {
            0 => RapierBodyParam::ContactSkin,
            1 => RapierBodyParam::Dominance,
            2 => RapierBodyParam::SoftCcd,
            3 => RapierBodyParam::Massless,
            4 => RapierBodyParam::AdditionalSolverIterations,
            _ => RapierBodyParam::ContactSkin,
        }
    }
}
#[macro_export]
macro_rules! make_rapier_server_godot_impl {
    ($class: ident) => {
        use godot::global::rid_allocate_id;
        use godot::global::rid_from_int64;
        #[cfg(feature = "serde-serialize")]
        use $crate::bodies::exportable_object::ExportableObject;
        #[cfg(feature = "serde-serialize")]
        use $crate::bodies::exportable_object::ObjectExportState;
        #[cfg(feature = "serde-serialize")]
        use $crate::bodies::exportable_object::ObjectImportState;
        use $crate::bodies::rapier_collision_object::IRapierCollisionObject;
        use $crate::fluids::rapier_fluid::RapierFluid;
        use $crate::joints::rapier_joint::IRapierJoint;
        use $crate::joints::rapier_joint::RapierJoint;
        use $crate::joints::rapier_joint_base::RapierJointType;
        use $crate::servers::rapier_physics_server_extra::RapierBodyParam;
        use $crate::servers::try_rapier_physics_server;
        #[godot_api]
        impl $class {
            #[constant]
            pub const BODY_PARAM_CONTACT_SKIN: i32 = 0;
            #[constant]
            pub const BODY_PARAM_DOMINANCE: i32 = 1;
            #[constant]
            pub const BODY_PARAM_MASSLESS: i32 = 3;
            #[constant]
            pub const BODY_PARAM_SOFT_CCD: i32 = 2;
            #[constant]
            pub const BODY_PARAM_ADDITIONAL_SOLVER_ITERATIONS: i32 = 4;
            #[constant]
            pub const JOINT_TYPE: i32 = 0;
            #[constant]
            pub const JOINT_TYPE_IMPULSE_JOINT: i32 = 0;
            /// Deprecated misspelling of [constant JOINT_TYPE_IMPULSE_JOINT].
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
            /// If [param param] is [member BODY_PARAM_MASSLESS] (3), sets if the body is massless or not.
            /// If [param param] is [member BODY_PARAM_ADDITIONAL_SOLVER_ITERATIONS] (4), sets extra solver iterations for this body.
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
            /// If [param param] is [member BODY_PARAM_MASSLESS] (3), gets if the body is massless or not.
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
            /// Set the contact force magnitude a contact must exceed before it is reported.
            /// Weaker contacts are dropped inside the solver, which is cheaper than filtering
            /// them in GDScript. A [param threshold] of `0.0` reports every contact, and is the
            /// default. Rapier uses the lower threshold of the two colliders in a pair.
            pub fn body_set_contact_force_threshold(body: Rid, threshold: real) {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get_mut(&body)
                    && let Some(body) = body.get_mut_body()
                {
                    body.set_contact_force_threshold(threshold, &mut physics_data.physics_engine);
                }
            }

            #[func]
            /// Get the contact force threshold set by [method body_set_contact_force_threshold].
            pub fn body_get_contact_force_threshold(body: Rid) -> real {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body)
                    && let Some(body) = body.get_body()
                {
                    return body.get_contact_force_threshold();
                }
                0.0
            }

            #[func]
            /// Get the friction impulse magnitude at a reported contact, which
            /// [method PhysicsDirectBodyState2D.get_contact_impulse] excludes because rapier
            /// solves friction on the contact tangent rather than the normal. A body scraping
            /// along a surface shows a large tangent impulse and a small normal one.
            /// Needs the same contact reporting as the built-in contact getters;
            /// [method body_get_total_friction_impulse] gives the body-wide total without it.
            pub fn body_get_contact_tangent_impulse(body: Rid, contact_idx: i32) -> real {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body)
                    && let Some(body) = body.get_body()
                {
                    return body.get_contact_tangent_impulse(contact_idx);
                }
                0.0
            }

            #[func]
            /// Get the total contact impulse acting on a body this step, read straight from the
            /// narrow phase. Needs no contact reporting, and is not truncated by
            /// [member RigidBody2D.max_contacts_reported].
            /// Divide by the step delta for a force.
            pub fn body_get_total_contact_impulse(body: Rid) -> Vector {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body)
                    && let Some(body) = body.get_body()
                {
                    let info = physics_data.physics_engine.body_get_contact_impulse(
                        body.get_base().get_space_id(),
                        body.get_base().get_body_handle(),
                        None,
                    );
                    return $crate::rapier_wrapper::convert::vector_to_godot(info.total_impulse);
                }
                Vector::ZERO
            }

            #[func]
            /// Get the strongest single contact impulse magnitude acting on a body this step.
            /// Useful for impact damage, where the peak matters more than the sum.
            pub fn body_get_max_contact_impulse(body: Rid) -> real {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body)
                    && let Some(body) = body.get_body()
                {
                    return physics_data
                        .physics_engine
                        .body_get_contact_impulse(
                            body.get_base().get_space_id(),
                            body.get_base().get_body_handle(),
                            None,
                        )
                        .max_impulse;
                }
                0.0
            }

            #[func]
            /// Get the total friction impulse magnitude acting on a body this step, which
            /// [method body_get_total_contact_impulse] excludes because rapier solves friction on
            /// the contact tangent rather than the normal. High friction with a low normal impulse
            /// means the body is scraping rather than being pressed. Needs no contact reporting.
            pub fn body_get_total_friction_impulse(body: Rid) -> real {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body)
                    && let Some(body) = body.get_body()
                {
                    return physics_data
                        .physics_engine
                        .body_get_contact_impulse(
                            body.get_base().get_space_id(),
                            body.get_base().get_body_handle(),
                            None,
                        )
                        .total_tangent_impulse;
                }
                0.0
            }

            #[func]
            /// Get the contact impulse [param body_a] receives from [param body_b] specifically.
            /// Returns a zero vector when the two are not touching.
            pub fn bodies_get_contact_impulse(body_a: Rid, body_b: Rid) -> Vector {
                let physics_data = physics_data();
                if let Some(object_a) = physics_data.collision_objects.get(&body_a)
                    && let Some(object_b) = physics_data.collision_objects.get(&body_b)
                    && let Some(body_a) = object_a.get_body()
                    && let Some(body_b) = object_b.get_body()
                {
                    let info = physics_data.physics_engine.body_get_contact_impulse(
                        body_a.get_base().get_space_id(),
                        body_a.get_base().get_body_handle(),
                        Some(body_b.get_base().get_body_handle()),
                    );
                    return $crate::rapier_wrapper::convert::vector_to_godot(info.total_impulse);
                }
                Vector::ZERO
            }

            #[func]
            /// Get the kinetic energy of a body, combining its linear and angular motion.
            pub fn body_get_kinetic_energy(body: Rid) -> real {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body)
                    && let Some(body) = body.get_body()
                {
                    return physics_data.physics_engine.body_get_kinetic_energy(
                        body.get_base().get_space_id(),
                        body.get_base().get_body_handle(),
                    );
                }
                0.0
            }

            #[func]
            /// Whether continuous collision detection actually ran for this body on the last
            /// step. Enabling CCD does not mean it engages every step; this reports when it did.
            pub fn body_is_ccd_active(body: Rid) -> bool {
                let physics_data = physics_data();
                if let Some(body) = physics_data.collision_objects.get(&body)
                    && let Some(body) = body.get_body()
                {
                    return physics_data.physics_engine.body_is_ccd_active(
                        body.get_base().get_space_id(),
                        body.get_base().get_body_handle(),
                    );
                }
                false
            }

            #[func]
            /// Get the world-space linear impulse a joint applied to hold its constraint on the
            /// last step, including its limit and motor contributions. Use it for breakable
            /// joints, rope tension or stress visualisation. Divide by the step delta for a force.
            /// Multibody joints resolve their constraints in reduced coordinates and return zero.
            pub fn joint_get_reaction_impulse(joint: Rid) -> Vector {
                let physics_data = physics_data();
                if let Some(joint) = physics_data.joints.get(&joint) {
                    let (linear, _) = physics_data.physics_engine.joint_get_reaction_impulse(
                        joint.get_base().get_space_id(),
                        joint.get_base().get_handle(),
                    );
                    return $crate::rapier_wrapper::convert::vector_to_godot(linear);
                }
                Vector::ZERO
            }

            #[func]
            /// Get the world-space angular impulse a joint applied to hold its constraint on the
            /// last step. See [method joint_get_reaction_impulse] for the linear counterpart.
            pub fn joint_get_reaction_angular_impulse(joint: Rid) -> Angle {
                let physics_data = physics_data();
                if let Some(joint) = physics_data.joints.get(&joint) {
                    let (_, angular) = physics_data.physics_engine.joint_get_reaction_impulse(
                        joint.get_base().get_space_id(),
                        joint.get_base().get_handle(),
                    );
                    return $crate::rapier_wrapper::convert::angle_to_godot(angular);
                }
                ANGLE_ZERO
            }

            #[func]
            /// Set an extra parameter for a joint.
            /// If [param param] is [member JOINT_TYPE] (0), sets if multibody or not.
            /// Use [member JOINT_TYPE_IMPULSE_JOINT] (0) for impulse joints, [member JOINT_TYPE_MULTIBODY_JOINT] (1) for multibody joints or [member JOINT_TYPE_MULTIBODY_KINEMATIC_JOINT] (2) for multibody kinematic joint.
            pub fn joint_set_extra_param(joint: Rid, param: i32, value: Variant) {
                if param == Self::JOINT_TYPE {
                    if let Ok(value) = value.try_to::<i32>() {
                        let joint_type = match value {
                            0 => RapierJointType::Impulse,
                            1 => RapierJointType::MultiBody,
                            2 => RapierJointType::MultiBodyKinematic,
                            _ => RapierJointType::Impulse, // default to Impulse
                        };
                        let Some(mut physics_singleton) = try_rapier_physics_server() else {
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
            /// Returns [member JOINT_TYPE_IMPULSE_JOINT] (0) for impulse joints, [member JOINT_TYPE_MULTIBODY_JOINT] (1) for multibody joints or [member JOINT_TYPE_MULTIBODY_KINEMATIC_JOINT] (2) for multibody kinematic joint.
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
            /// This automatically applies the computed joint displacements.
            pub fn joint_solve_inverse_kinematics(joint: Rid, target_transform: Transform) {
                use $crate::rapier_wrapper::convert::vector_to_rapier;
                let physics_data = physics_data();
                if let Some(joint_obj) = physics_data.joints.get_mut(&joint) {
                    let space_handle = joint_obj.get_base().get_space_id();
                    let joint_handle = joint_obj.get_base().get_handle();
                    let custom_ik_options = joint_obj.get_base().custom_ik_options;
                    let target_isometry = {
                        let position = vector_to_rapier(target_transform.origin);
                        let rotation = transform_rotation_rapier(&target_transform);
                        rapier::prelude::Pose::from_parts(position, rotation)
                    };
                    physics_data.physics_engine.multibody_solve_ik(
                        space_handle,
                        joint_handle,
                        target_isometry,
                        custom_ik_options,
                    );
                    for body in physics_data
                        .physics_engine
                        .get_multibody_rigidbodies(space_handle, joint_handle)
                    {
                        physics_data
                            .physics_engine
                            .body_wake_up(space_handle, body, true);
                    }
                }
            }

            #[func]
            /// Set custom IK options for a specific joint.
            /// This overrides the default Rapier IK parameters.
            /// constrained_axes: bitmask for which axes to constrain (1=X/Lin, 2=Y/Lin, 4=Z/Lin, 8=AngX, 16=AngY, 32=AngZ)
            ///   Common values: 3=XY position (2D), 7=XYZ position (3D), 56=rotation (3D), 63=all (3D)
            /// Default values: damping=1.0, max_iterations=10, constrained_axes=63, epsilon_linear=0.001, epsilon_angular=0.001
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
                        damping: damping.max(0.0),
                        max_iters: max_iterations.max(0) as usize,
                        constrained_axes: JointAxesMask::from_bits_truncate(
                            constrained_axes.max(0) as u8,
                        ),
                        epsilon_linear: epsilon_linear.max(0.0),
                        epsilon_angular: epsilon_angular.max(0.0),
                    };
                    joint_obj.get_mut_base().custom_ik_options = options;
                }
            }

            #[func]
            /// Reset IK options to Rapier's default values.
            /// Default values: damping=1.0, max_iterations=10, constrained_axes=63, epsilon_linear=0.001, epsilon_angular=0.001
            pub fn joint_reset_ik_options(joint: Rid) {
                let physics_data = physics_data();
                if let Some(joint_obj) = physics_data.joints.get_mut(&joint) {
                    use rapier::dynamics::InverseKinematicsOption;
                    joint_obj.get_mut_base().custom_ik_options = InverseKinematicsOption::default();
                }
            }

            #[func]
            pub fn joint_set_motor_position_options(
                joint: Rid,
                target_pos: real,
                stiffness: real,
                damping: real,
                enabled: bool,
            ) {
                let physics_data = physics_data();
                if let Some(RapierJoint::RapierRevoluteJoint(revolute)) =
                    physics_data.joints.get_mut(&joint)
                {
                    revolute.set_motor_position_options(
                        &mut physics_data.physics_engine,
                        target_pos,
                        stiffness,
                        damping,
                        enabled,
                    )
                }
            }

            #[cfg(feature = "serde-serialize")]
            pub fn fetch_state_internal<'a>(physics_object: Rid) -> Option<ObjectExportState<'a>> {
                let physics_data = physics_data();
                use $crate::joints::rapier_joint::IRapierJoint;
                use $crate::shapes::rapier_shape::IRapierShape;
                if let Some(body) = physics_data.collision_objects.get(&physics_object) {
                    return body.get_export_state(&mut physics_data.physics_engine);
                } else if let Some(joint) = physics_data.joints.get(&physics_object) {
                    return joint
                        .get_base()
                        .get_export_state(&mut physics_data.physics_engine)
                        .map(ObjectExportState::JointBase);
                } else if let Some(shape) = physics_data.shapes.get(&physics_object) {
                    return shape
                        .get_base()
                        .get_export_state(&mut physics_data.physics_engine)
                        .map(ObjectExportState::ShapeBase);
                } else if let Some(space) = physics_data.spaces.get(&physics_object) {
                    return space
                        .get_export_state(&mut physics_data.physics_engine)
                        .map(ObjectExportState::Space);
                }
                None
            }

            #[cfg(feature = "serde-serialize")]
            pub fn load_state_internal(physics_object: Rid, data: ObjectImportState) {
                let physics_data = physics_data();
                use $crate::joints::rapier_joint::IRapierJoint;
                use $crate::servers::rapier_physics_singleton::insert_id_rid;
                use $crate::servers::rapier_physics_singleton::remove_id_rid;
                use $crate::shapes::rapier_shape::IRapierShape;
                if let Some(body) = physics_data.collision_objects.get_mut(&physics_object) {
                    remove_id_rid(body.get_base().get_id(), &mut physics_data.ids);
                    body.import_state(&mut physics_data.physics_engine, data);
                    insert_id_rid(
                        body.get_base().get_id(),
                        body.get_base().get_rid(),
                        &mut physics_data.ids,
                    );
                    return;
                } else if let Some(joint) = physics_data.joints.get_mut(&physics_object) {
                    remove_id_rid(joint.get_base().get_id(), &mut physics_data.ids);
                    joint
                        .get_mut_base()
                        .import_state(&mut physics_data.physics_engine, data);
                    insert_id_rid(
                        joint.get_base().get_id(),
                        joint.get_base().get_rid(),
                        &mut physics_data.ids,
                    );
                    return;
                } else if let Some(shape) = physics_data.shapes.get_mut(&physics_object) {
                    remove_id_rid(shape.get_base().get_id(), &mut physics_data.ids);
                    // Recreate shape handle:
                    shape
                        .get_mut_base()
                        .destroy_shape(&mut physics_data.physics_engine);
                    shape
                        .get_mut_base()
                        .import_state(&mut physics_data.physics_engine, data);
                    insert_id_rid(
                        shape.get_base().get_id(),
                        shape.get_base().get_rid(),
                        &mut physics_data.ids,
                    );
                    return;
                } else if let Some(space) = physics_data.spaces.get_mut(&physics_object) {
                    return space.import_state(&mut physics_data.physics_engine, data);
                }
            }

            #[func]
            /// Set how rigidly [param joint] holds its constraint.
            ///
            /// Higher [param natural_frequency] is stiffer; [param damping_ratio] 1 is
            /// critically damped.
            pub fn joint_set_softness(joint: Rid, natural_frequency: real, damping_ratio: real) {
                let physics_data = physics_data();
                if let Some(joint) = physics_data.joints.get(&joint) {
                    physics_data.physics_engine.joint_set_softness(
                        joint.get_base().get_space_id(),
                        joint.get_base().get_handle(),
                        natural_frequency,
                        damping_ratio,
                    );
                }
            }

            #[func]
            /// Enable or disable [param joint] without destroying it.
            pub fn joint_set_enabled(joint: Rid, enabled: bool) {
                let physics_data = physics_data();
                if let Some(joint) = physics_data.joints.get(&joint) {
                    physics_data.physics_engine.joint_set_enabled(
                        joint.get_base().get_space_id(),
                        joint.get_base().get_handle(),
                        enabled,
                    );
                }
            }

            #[func]
            /// Turn [param joint] into a rope joint keeping the two bodies within
            /// [param max_distance] of each other.
            ///
            /// Anchors are in world space.
            pub fn joint_make_rope(
                joint: Rid,
                anchor_a: Vector,
                anchor_b: Vector,
                max_distance: real,
                body_a: Rid,
                body_b: Rid,
            ) {
                let physics_data = physics_data();
                let Some(mut physics_singleton) = try_rapier_physics_server() else {
                    return;
                };
                let joint_type = if let Some(prev_joint) = physics_data.joints.get(&joint) {
                    prev_joint.get_base().get_joint_type()
                } else {
                    $crate::joints::rapier_joint_base::RapierJointType::Impulse
                };
                if let Some(body_a) = physics_data.collision_objects.get(&body_a)
                    && let Some(body_b) = physics_data.collision_objects.get(&body_b)
                {
                    let id = physics_singleton.bind_mut().implementation.next_id();
                    let new_joint = RapierJoint::RapierRopeJoint(
                        $crate::joints::rapier_rope_joint::RapierRopeJoint::new(
                            id,
                            joint,
                            anchor_a,
                            anchor_b,
                            max_distance,
                            body_a,
                            body_b,
                            &mut physics_data.physics_engine,
                            joint_type,
                        ),
                    );
                    if let Some(mut prev_joint) = physics_data.joints.insert(joint, new_joint) {
                        prev_joint
                            .get_mut_base()
                            .destroy_joint(&mut physics_data.physics_engine);
                    }
                }
            }

            #[func]
            /// Turn [param joint] into a fixed joint locking all relative motion between
            /// [param body_a] and [param body_b].
            ///
            /// The anchor is in world space.
            pub fn joint_make_fixed(joint: Rid, anchor: Vector, body_a: Rid, body_b: Rid) {
                let physics_data = physics_data();
                let Some(mut physics_singleton) = try_rapier_physics_server() else {
                    return;
                };
                let joint_type = if let Some(prev_joint) = physics_data.joints.get(&joint) {
                    prev_joint.get_base().get_joint_type()
                } else {
                    $crate::joints::rapier_joint_base::RapierJointType::Impulse
                };
                if let Some(body_a) = physics_data.collision_objects.get(&body_a)
                    && let Some(body_b) = physics_data.collision_objects.get(&body_b)
                {
                    let id = physics_singleton.bind_mut().implementation.next_id();
                    let new_joint = RapierJoint::RapierFixedJoint(
                        $crate::joints::rapier_fixed_joint::RapierFixedJoint::new(
                            id,
                            joint,
                            anchor,
                            anchor,
                            body_a,
                            body_b,
                            &mut physics_data.physics_engine,
                            joint_type,
                        ),
                    );
                    if let Some(mut prev_joint) = physics_data.joints.insert(joint, new_joint) {
                        prev_joint
                            .get_mut_base()
                            .destroy_joint(&mut physics_data.physics_engine);
                    }
                }
            }

            #[func]
            /// Create a new fluid.
            pub(crate) fn fluid_create() -> Rid {
                let physics_data = physics_data();
                let rid = rid_from_int64(rid_allocate_id());
                let Some(mut physics_singleton) = try_rapier_physics_server() else {
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
                let Some(physics_singleton) = try_rapier_physics_server() else {
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
                let Some(physics_singleton) = try_rapier_physics_server() else {
                    return Array::default();
                };
                return physics_singleton
                    .bind()
                    .implementation
                    .space_get_bodies_transform(space, bodies);
            }

            #[func]
            /// Step the space forward.
            pub fn space_step(space: Rid, delta: f32) {
                let Some(mut physics_singleton) = try_rapier_physics_server() else {
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
                let Some(physics_singleton) = try_rapier_physics_server() else {
                    return 0;
                };
                return physics_singleton.bind().implementation.get_id(rid) as i64;
            }

            #[func]
            /// Get the global id of the physics server.
            fn get_global_id() -> i64 {
                let Some(physics_singleton) = try_rapier_physics_server() else {
                    return 0;
                };
                return physics_singleton.bind().implementation.id as i64;
            }

            #[func]
            /// Set the global id of the physics server.
            pub fn set_global_id(id: i64) {
                let Some(mut physics_singleton) = try_rapier_physics_server() else {
                    return;
                };
                physics_singleton.bind_mut().implementation.id = id as u64;
            }

            #[func]
            /// Get the stats of the physics server.
            fn get_stats() -> VarDictionary {
                let mut dictionary = VarDictionary::new();
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
