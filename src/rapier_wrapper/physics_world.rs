use std::num::NonZeroUsize;
use std::sync::mpsc;

use hashbrown::HashMap;
use rapier::data::Index;
use rapier::parry::utils::IsometryOpt;
use rapier::prelude::*;
use salva::integrations::rapier::FluidsPipeline;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsCollisionObjects;
use crate::servers::rapier_physics_singleton::PhysicsIds;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::spaces::rapier_space::RapierSpace;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(PartialEq, Eq, Clone, Copy, Debug, Default)]
pub struct JointHandle {
    pub index: Index,
    pub kinematic: bool,
    pub multibody: bool,
}
pub struct ActiveBodyInfo {
    pub body_user_data: UserData,
}
pub struct BeforeActiveBodyInfo {
    pub body_user_data: UserData,
    pub previous_velocity: Vector<Real>,
}
#[derive(Default)]
pub struct ContactPointInfo {
    pub pixel_local_pos_1: Vector<Real>,
    pub pixel_local_pos_2: Vector<Real>,
    pub pixel_velocity_pos_1: Vector<Real>,
    pub pixel_velocity_pos_2: Vector<Real>,
    pub normal: Vector<Real>,
    pub pixel_distance: Real,
    pub pixel_impulse: Real,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(PartialEq, Eq, Clone, Copy, Debug, Default)]
pub struct CollisionEventInfo {
    pub collider1: ColliderHandle,
    pub collider2: ColliderHandle,
    pub user_data1: UserData,
    pub user_data2: UserData,
    pub is_sensor: bool,
    pub is_started: bool,
    pub is_stopped: bool,
    pub is_removed: bool,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct ContactForceEventInfo {
    pub user_data1: UserData,
    pub user_data2: UserData,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct PhysicsObjects {
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,

    pub collider_set: ColliderSet,
    pub rigid_body_set: RigidBodySet,

    pub removed_rigid_bodies_user_data: HashMap<RigidBodyHandle, UserData>,
    pub removed_colliders_user_data: HashMap<ColliderHandle, UserData>,

    pub handle: WorldHandle,
}
pub struct PhysicsWorld {
    pub physics_objects: PhysicsObjects,
    pub physics_pipeline: PhysicsPipeline,
    pub fluids_pipeline: FluidsPipeline,
    #[cfg(feature = "parallel")]
    pub thread_pool: rapier::rayon::ThreadPool,
}
impl PhysicsWorld {
    pub fn new(settings: &WorldSettings) -> PhysicsWorld {
        let mut physics_pipeline = PhysicsPipeline::new();
        if settings.counters_enabled {
            physics_pipeline.counters.enable();
        }
        PhysicsWorld {
            physics_objects: PhysicsObjects {
                island_manager: IslandManager::new(),
                broad_phase: DefaultBroadPhase::new(),
                narrow_phase: NarrowPhase::new(),
                impulse_joint_set: ImpulseJointSet::new(),
                multibody_joint_set: MultibodyJointSet::new(),
                ccd_solver: CCDSolver::new(),

                rigid_body_set: RigidBodySet::new(),
                collider_set: ColliderSet::new(),

                removed_rigid_bodies_user_data: HashMap::new(),
                removed_colliders_user_data: HashMap::new(),

                handle: WorldHandle::default(),
            },
            physics_pipeline,
            fluids_pipeline: FluidsPipeline::new_with_boundary_coef(
                settings.particle_radius,
                settings.smoothing_factor,
                settings.boundary_coef,
            ),
            #[cfg(feature = "parallel")]
            thread_pool: rapier::rayon::ThreadPoolBuilder::new()
                .num_threads(settings.thread_count)
                .build()
                .unwrap(),
        }
    }

    pub fn step(
        &mut self,
        settings: &SimulationSettings,
        collision_filter_body_callback: CollisionFilterCallback,
        collision_modify_contacts_callback: CollisionModifyContactsCallback,
        space: &mut RapierSpace,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        for handle in self.physics_objects.island_manager.active_bodies() {
            if let Some(body) = self.physics_objects.rigid_body_set.get(*handle) {
                let before_active_body_info = BeforeActiveBodyInfo {
                    body_user_data: self.get_rigid_body_user_data(*handle),
                    previous_velocity: *body.linvel(),
                };
                space.before_active_body_callback(
                    &before_active_body_info,
                    physics_collision_objects,
                    physics_ids,
                );
            }
        }
        let mut integration_parameters = IntegrationParameters {
            length_unit: settings.length_unit,
            dt: settings.dt,
            contact_damping_ratio: settings.contact_damping_ratio,
            contact_natural_frequency: settings.contact_natural_frequency,
            max_ccd_substeps: settings.max_ccd_substeps,
            normalized_allowed_linear_error: settings.normalized_allowed_linear_error,
            normalized_max_corrective_velocity: settings.normalized_max_corrective_velocity,
            normalized_prediction_distance: settings.normalized_prediction_distance,
            num_internal_stabilization_iterations: settings.num_internal_stabilization_iterations,
            ..Default::default()
        };
        if let Some(iterations) = NonZeroUsize::new(settings.num_solver_iterations) {
            integration_parameters.num_solver_iterations = iterations.into();
        }
        integration_parameters.num_internal_pgs_iterations = settings.num_internal_pgs_iterations;
        let gravity = settings.pixel_gravity;
        let liquid_gravity = settings.pixel_liquid_gravity;
        let physics_hooks = PhysicsHooksCollisionFilter {
            collision_filter_body_callback: &collision_filter_body_callback,
            collision_modify_contacts_callback: &collision_modify_contacts_callback,
            physics_collision_objects,
            physics_ids,
            last_step: RapierSpace::get_last_step(),
            ghost_collision_distance: space.get_ghost_collision_distance(),
        };
        // Initialize the event collector.
        let (collision_send, collision_recv) = mpsc::channel();
        let (contact_force_send, contact_force_recv) = mpsc::channel();
        let event_handler = ContactEventHandler::new(collision_send, contact_force_send);
        #[cfg(feature = "parallel")]
        {
            let physics_pipeline = &mut self.physics_pipeline;
            self.thread_pool.install(|| {
                physics_pipeline.step(
                    &gravity,
                    &integration_parameters,
                    &mut self.physics_objects.island_manager,
                    &mut self.physics_objects.broad_phase,
                    &mut self.physics_objects.narrow_phase,
                    &mut self.physics_objects.rigid_body_set,
                    &mut self.physics_objects.collider_set,
                    &mut self.physics_objects.impulse_joint_set,
                    &mut self.physics_objects.multibody_joint_set,
                    &mut self.physics_objects.ccd_solver,
                    &physics_hooks,
                    &event_handler,
                );
            });
        }
        #[cfg(not(feature = "parallel"))]
        self.physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut self.physics_objects.island_manager,
            &mut self.physics_objects.broad_phase,
            &mut self.physics_objects.narrow_phase,
            &mut self.physics_objects.rigid_body_set,
            &mut self.physics_objects.collider_set,
            &mut self.physics_objects.impulse_joint_set,
            &mut self.physics_objects.multibody_joint_set,
            &mut self.physics_objects.ccd_solver,
            &physics_hooks,
            &event_handler,
        );
        if self.fluids_pipeline.liquid_world.fluids().len() > 0 {
            self.fluids_pipeline.step(
                &liquid_gravity,
                integration_parameters.dt,
                &self.physics_objects.collider_set,
                &mut self.physics_objects.rigid_body_set,
            );
        }
        for handle in self.physics_objects.island_manager.active_bodies() {
            let active_body_info = ActiveBodyInfo {
                body_user_data: self.get_rigid_body_user_data(*handle),
            };
            space.active_body_callback(&active_body_info, physics_collision_objects, physics_ids);
        }
        while let Ok(collision_event) = collision_recv.try_recv() {
            let handle1 = collision_event.collider1();
            let handle2 = collision_event.collider2();
            // Handle the collision event.
            let event_info = CollisionEventInfo {
                is_sensor: collision_event.sensor(),
                is_removed: collision_event.removed(),
                is_started: collision_event.started(),
                is_stopped: collision_event.stopped(),
                collider1: handle1,
                collider2: handle2,
                user_data1: self.get_collider_user_data(handle1),
                user_data2: self.get_collider_user_data(handle2),
            };
            space.collision_event_callback(&event_info, physics_collision_objects, physics_ids);
        }
        while let Ok(contact_pair) = contact_force_recv.try_recv() {
            if let Some(collider1) = self
                .physics_objects
                .collider_set
                .get(contact_pair.collider1)
                && let Some(collider2) = self
                    .physics_objects
                    .collider_set
                    .get(contact_pair.collider2)
            {
                // Handle the contact force event.
                let event_info = ContactForceEventInfo {
                    user_data1: UserData::new(collider1.user_data),
                    user_data2: UserData::new(collider2.user_data),
                };
                let send_contact_points = space.contact_force_event_callback(
                    &event_info,
                    physics_collision_objects,
                    physics_ids,
                );
                if send_contact_points
                    && let Some(body1) = self.get_collider_rigid_body(collider1)
                    && let Some(body2) = self.get_collider_rigid_body(collider2)
                {
                    // Helper closure to send contact point info
                    let mut send_contact_point =
                        |manifold_normal: Vector<Real>,
                         contact_point: &TrackedContact<ContactData>| {
                            let collider_pos_1 = *collider1.position();
                            let collider_pos_2 = *collider2.position();
                            let point_velocity_1 = body1
                                .velocity_at_point(&Point::from(collider_pos_1.translation.vector));
                            let point_velocity_2 = body2
                                .velocity_at_point(&Point::from(collider_pos_2.translation.vector));
                            let pixel_pos_1 = collider_pos_1.translation.vector;
                            let pixel_pos_2 = collider_pos_2.translation.vector;
                            let contact_info = ContactPointInfo {
                                normal: manifold_normal,
                                pixel_local_pos_1: pixel_pos_1
                                    + (body1
                                        .rotation()
                                        .transform_vector(&contact_point.local_p1.coords)),
                                pixel_local_pos_2: pixel_pos_2
                                    + (body2
                                        .rotation()
                                        .transform_vector(&contact_point.local_p2.coords)),
                                pixel_velocity_pos_1: point_velocity_1,
                                pixel_velocity_pos_2: point_velocity_2,
                                pixel_distance: contact_point.dist,
                                pixel_impulse: contact_point.data.impulse,
                            };
                            space.contact_point_callback(
                                &contact_info,
                                &event_info,
                                physics_collision_objects,
                                physics_ids,
                            );
                        };
                    // Find the contact pair, if it exists, between two colliders
                    let mut has_any_valid_contact = false;
                    // We may also read the contact manifolds to access the contact geometry.
                    for manifold in &contact_pair.manifolds {
                        let manifold_normal = manifold.data.normal;
                        for contact_point in &manifold.points {
                            // Any given "contact point" may actually be a predictive point-- these points do not actually represent a contact for this frame.
                            // To prune out these false contacts, the below is a direct port of Rapier's own logic (from src/geometry/narrow_phase.rs).
                            // This ensures that we prune out the same contacts that Rapier's contact solving prunes.
                            // However, ideally we wouldn't have to duplicate this logic, and instead Rapier would expose some mechanism to only push the verified contacts.
                            let effective_contact_dist = contact_point.dist
                                - collider1.contact_skin()
                                - collider2.contact_skin();
                            let world_pos1 =
                                manifold.subshape_pos1.prepend_to(collider1.position());
                            let world_pos2 =
                                manifold.subshape_pos2.prepend_to(collider2.position());
                            let keep_solver_contact = effective_contact_dist
                                < settings.predictive_contact_allowance_threshold
                                    * settings.length_unit
                                || {
                                    let world_pt1 = world_pos1 * contact_point.local_p1;
                                    let world_pt2 = world_pos2 * contact_point.local_p2;
                                    let vel1 = self
                                        .get_collider_rigid_body(collider1)
                                        .map(|rb| rb.velocity_at_point(&world_pt1))
                                        .unwrap_or_default();
                                    let vel2 = self
                                        .get_collider_rigid_body(collider2)
                                        .map(|rb| rb.velocity_at_point(&world_pt2))
                                        .unwrap_or_default();
                                    effective_contact_dist
                                        + (vel2 - vel1).dot(&manifold.data.normal) * settings.dt
                                        < settings.predictive_contact_allowance_threshold
                                            * settings.length_unit
                                };
                            if keep_solver_contact {
                                has_any_valid_contact = true;
                                send_contact_point(manifold_normal, contact_point);
                            }
                        }
                    }
                    // If no valid contacts were found, call callback with first contact
                    if !has_any_valid_contact
                        && !contact_pair.manifolds.is_empty()
                        && let Some(manifold) = contact_pair.manifolds.first()
                        && let Some(contact_point) = manifold.points.first()
                    {
                        send_contact_point(manifold.data.normal, contact_point);
                    }
                }
            }
        }
        // remove all the removed colliders and rigidbodies user data
        self.physics_objects.removed_rigid_bodies_user_data.clear();
        self.physics_objects.removed_colliders_user_data.clear();
    }

    pub fn insert_collider(
        &mut self,
        collider: Collider,
        body_handle: RigidBodyHandle,
    ) -> ColliderHandle {
        if body_handle != RigidBodyHandle::invalid() {
            let rigid_body_handle = body_handle;
            self.physics_objects.collider_set.insert_with_parent(
                collider,
                rigid_body_handle,
                &mut self.physics_objects.rigid_body_set,
            )
        } else {
            self.physics_objects.collider_set.insert(collider)
        }
    }

    pub fn remove_collider(&mut self, collider_handle: ColliderHandle) {
        if let Some(collider) = self.physics_objects.collider_set.remove(
            collider_handle,
            &mut self.physics_objects.island_manager,
            &mut self.physics_objects.rigid_body_set,
            false,
        ) {
            self.physics_objects
                .removed_colliders_user_data
                .insert(collider_handle, UserData::new(collider.user_data));
        }
    }

    pub fn get_collider_user_data(&self, collider_handle: ColliderHandle) -> UserData {
        let collider = self.physics_objects.collider_set.get(collider_handle);
        if let Some(collider) = collider {
            return UserData::new(collider.user_data);
        }
        // removed collider
        if let Some(user_data) = self
            .physics_objects
            .removed_colliders_user_data
            .get(&collider_handle)
        {
            return *user_data;
        }
        UserData::invalid_user_data()
    }

    pub fn get_collider_rigid_body(&self, collider: &Collider) -> Option<&RigidBody> {
        let parent = collider.parent();
        if let Some(parent) = parent {
            return self.physics_objects.rigid_body_set.get(parent);
        }
        None
    }

    pub fn remove_rigid_body(&mut self, body_handle: RigidBodyHandle) {
        let rigid_body_handle = body_handle;
        if let Some(rigid_body) = self.physics_objects.rigid_body_set.remove(
            rigid_body_handle,
            &mut self.physics_objects.island_manager,
            &mut self.physics_objects.collider_set,
            &mut self.physics_objects.impulse_joint_set,
            &mut self.physics_objects.multibody_joint_set,
            true,
        ) {
            self.physics_objects
                .removed_rigid_bodies_user_data
                .insert(rigid_body_handle, UserData::new(rigid_body.user_data));
        }
    }

    pub fn get_rigid_body_user_data(&self, rigid_body_handle: RigidBodyHandle) -> UserData {
        let rigid_body = self.physics_objects.rigid_body_set.get(rigid_body_handle);
        if let Some(rigid_body) = rigid_body {
            return UserData::new(rigid_body.user_data);
        }
        // removed rigidbody
        if let Some(user_data) = self
            .physics_objects
            .removed_rigid_bodies_user_data
            .get(&rigid_body_handle)
        {
            return *user_data;
        }
        UserData::invalid_user_data()
    }

    pub fn insert_joint(
        &mut self,
        body_handle_1: RigidBodyHandle,
        body_handle_2: RigidBodyHandle,
        multibody: bool,
        kinematic: bool,
        joint: impl Into<GenericJoint>,
    ) -> JointHandle {
        let rigid_body_1_handle = body_handle_1;
        let rigid_body_2_handle = body_handle_2;
        match (multibody, kinematic) {
            (false, _) => {
                let impulse_joint_handle = self.physics_objects.impulse_joint_set.insert(
                    rigid_body_1_handle,
                    rigid_body_2_handle,
                    joint,
                    true,
                );
                return JointHandle {
                    index: impulse_joint_handle.0,
                    kinematic,
                    multibody,
                };
            }
            (true, true) => {
                let multibody_joint_handle = self
                    .physics_objects
                    .multibody_joint_set
                    .insert_kinematic(rigid_body_1_handle, rigid_body_2_handle, joint, true);
                if let Some(multibody_joint_handle) = multibody_joint_handle {
                    return JointHandle {
                        index: multibody_joint_handle.0,
                        kinematic,
                        multibody,
                    };
                }
            }
            (true, false) => {
                let multibody_joint_handle = self.physics_objects.multibody_joint_set.insert(
                    rigid_body_1_handle,
                    rigid_body_2_handle,
                    joint,
                    true,
                );
                if let Some(multibody_joint_handle) = multibody_joint_handle {
                    return JointHandle {
                        index: multibody_joint_handle.0,
                        kinematic,
                        multibody,
                    };
                }
            }
        }
        JointHandle::default()
    }

    pub fn get_mut_joint(&mut self, handle: JointHandle) -> Option<&mut GenericJoint> {
        match handle.multibody {
            false => {
                let joint = self
                    .physics_objects
                    .impulse_joint_set
                    .get_mut(ImpulseJointHandle(handle.index), true);
                if let Some(joint) = joint {
                    return Some(&mut joint.data);
                }
            }
            true => {
                let joint = self
                    .physics_objects
                    .multibody_joint_set
                    .get_mut(MultibodyJointHandle(handle.index));
                if let Some((multibody, link_id)) = joint
                    && let Some(link) = multibody.link_mut(link_id)
                {
                    return Some(&mut link.joint.data);
                }
            }
        }
        None
    }

    // TODO multibody joints
    /*
    pub fn get_joint(&self, handle: JointHandle) -> Option<&GenericJoint> {
        match handle.multibody {
            false => {
                let joint = self
                    .physics_objects
                    .impulse_joint_set
                    .get(ImpulseJointHandle(handle.index));
                if let Some(joint) = joint {
                    return Some(&joint.data);
                }
            }
            true => {
                let joint = self
                    .physics_objects
                    .multibody_joint_set
                    .get(MultibodyJointHandle(handle.index));
                if let Some((multibody, link_id)) = joint
                    && let Some(link) = multibody.link(link_id)
                {
                    return Some(&link.joint.data);
                }
            }
        }
        None
    } */
    pub fn get_impulse_joint(&self, handle: JointHandle) -> Option<&ImpulseJoint> {
        match handle.multibody {
            false => self
                .physics_objects
                .impulse_joint_set
                .get(ImpulseJointHandle(handle.index)),
            true => None,
        }
    }

    pub fn remove_joint(&mut self, handle: JointHandle) {
        match handle.multibody {
            false => {
                let joint_handle = handle;
                self.physics_objects
                    .impulse_joint_set
                    .remove(ImpulseJointHandle(joint_handle.index), true);
            }
            true => {
                let joint_handle = handle;
                self.physics_objects
                    .multibody_joint_set
                    .remove(MultibodyJointHandle(joint_handle.index), true);
            }
        }
    }
}
#[derive(Default)]
pub struct PhysicsEngine {
    pub physics_worlds: HashMap<RapierId, PhysicsWorld>,
    pub shapes: HashMap<RapierId, SharedShape>,
}
impl PhysicsEngine {
    pub fn get_mut_world(&mut self, world_handle: WorldHandle) -> Option<&mut PhysicsWorld> {
        self.physics_worlds.get_mut(&world_handle)
    }

    pub fn get_world(&self, world_handle: WorldHandle) -> Option<&PhysicsWorld> {
        self.physics_worlds.get(&world_handle)
    }

    pub fn insert_shape(&mut self, shape: SharedShape, handle: ShapeHandle) {
        self.shapes.insert(handle, shape);
    }

    pub fn remove_shape(&mut self, shape_handle: ShapeHandle) {
        self.shapes.remove_entry(&shape_handle);
    }

    pub fn get_shape(&self, shape_handle: ShapeHandle) -> Option<&SharedShape> {
        self.shapes.get(&shape_handle)
    }

    pub fn world_create(&mut self, settings: &WorldSettings, handle: WorldHandle) {
        let mut physics_world = PhysicsWorld::new(settings);
        physics_world.physics_objects.handle = handle;
        self.physics_worlds.insert(handle, physics_world);
    }

    pub fn world_destroy(&mut self, world_handle: WorldHandle) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.physics_objects.handle = WorldHandle::default();
            self.physics_worlds.remove(&world_handle);
        }
    }

    pub fn world_reset_if_empty(&mut self, world_handle: WorldHandle, settings: &WorldSettings) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && physics_world.physics_objects.impulse_joint_set.is_empty()
            && physics_world
                .physics_objects
                .multibody_joint_set
                .multibodies()
                .peekable()
                .peek()
                .is_none()
            && physics_world.physics_objects.rigid_body_set.is_empty()
            && physics_world.physics_objects.collider_set.is_empty()
        {
            let new_physics_world = PhysicsWorld::new(settings);
            physics_world.fluids_pipeline = new_physics_world.fluids_pipeline;
            physics_world.physics_pipeline = new_physics_world.physics_pipeline;
            physics_world.physics_objects = new_physics_world.physics_objects;
        }
    }

    pub fn world_get_active_objects_count(&mut self, world_handle: WorldHandle) -> usize {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            return physics_world
                .physics_objects
                .island_manager
                .active_bodies()
                .len();
        }
        0
    }

    pub fn world_export(&mut self, world_handle: WorldHandle) -> Option<&PhysicsObjects> {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            return Some(&physics_world.physics_objects);
        }
        None
    }

    pub fn world_import(
        &mut self,
        world_handle: WorldHandle,
        settings: &WorldSettings,
        physics_objects: PhysicsObjects,
    ) {
        let mut physics_world = PhysicsWorld::new(settings);
        physics_world.physics_objects = physics_objects;
        self.physics_worlds.insert(world_handle, physics_world);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn world_step(
        &mut self,
        world_handle: WorldHandle,
        settings: &SimulationSettings,
        collision_filter_body_callback: CollisionFilterCallback,
        collision_modify_contacts_callback: CollisionModifyContactsCallback,
        space: &mut RapierSpace,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.step(
                settings,
                collision_filter_body_callback,
                collision_modify_contacts_callback,
                space,
                physics_collision_objects,
                physics_ids,
            );
        }
    }
}
