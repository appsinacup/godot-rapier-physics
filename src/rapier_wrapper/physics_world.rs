use std::num::NonZeroUsize;

use godot::global::godot_print;
use rapier::crossbeam;
use rapier::data::Arena;
use rapier::data::Index;
use rapier::prelude::*;
use salva::integrations::rapier::FluidsPipeline;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsCollisionObjects;
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
    pub pixel_tangent_impulse: TangentImpulse<Real>,
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
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub query_pipeline: QueryPipeline,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub island_manager: IslandManager,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub broad_phase: BroadPhaseMultiSap,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub ccd_solver: CCDSolver,

    pub collider_set: ColliderSet,
    pub rigid_body_set: RigidBodySet,

    pub handle: WorldHandle,
}
pub struct PhysicsWorld {
    pub physics_objects: PhysicsObjects,
    pub physics_pipeline: PhysicsPipeline,
    pub fluids_pipeline: FluidsPipeline,
}
impl PhysicsWorld {
    pub fn new(settings: &WorldSettings) -> PhysicsWorld {
        let mut physics_pipeline = PhysicsPipeline::new();
        if settings.counters_enabled {
            physics_pipeline.counters.enable();
        }
        PhysicsWorld {
            physics_objects: PhysicsObjects {
                query_pipeline: QueryPipeline::new(),
                island_manager: IslandManager::new(),
                broad_phase: DefaultBroadPhase::new(),
                narrow_phase: NarrowPhase::new(),
                impulse_joint_set: ImpulseJointSet::new(),
                multibody_joint_set: MultibodyJointSet::new(),
                ccd_solver: CCDSolver::new(),

                rigid_body_set: RigidBodySet::new(),
                collider_set: ColliderSet::new(),

                handle: WorldHandle::default(),
            },
            physics_pipeline,
            fluids_pipeline: FluidsPipeline::new(
                settings.particle_radius,
                settings.smoothing_factor,
            ),
        }
    }

    pub fn step(
        &mut self,
        settings: &SimulationSettings,
        collision_filter_body_callback: CollisionFilterCallback,
        collision_modify_contacts_callback: CollisionModifyContactsCallback,
        space: &mut RapierSpace,
        physics_collision_objects: &mut PhysicsCollisionObjects,
    ) {
        for handle in self.physics_objects.island_manager.active_dynamic_bodies() {
            if let Some(body) = self.physics_objects.rigid_body_set.get(*handle) {
                let before_active_body_info = BeforeActiveBodyInfo {
                    body_user_data: self.get_rigid_body_user_data(*handle),
                    previous_velocity: *body.linvel(),
                };
                space.before_active_body_callback(
                    &before_active_body_info,
                    physics_collision_objects,
                );
            }
        }
        let mut integration_parameters = IntegrationParameters {
            length_unit: settings.length_unit,
            dt: settings.dt,
            contact_damping_ratio: settings.contact_damping_ratio,
            contact_natural_frequency: settings.contact_natural_frequency,
            max_ccd_substeps: settings.max_ccd_substeps,
            joint_damping_ratio: settings.joint_damping_ratio,
            joint_natural_frequency: settings.joint_natural_frequency,
            normalized_allowed_linear_error: settings.normalized_allowed_linear_error,
            normalized_max_corrective_velocity: settings.normalized_max_corrective_velocity,
            normalized_prediction_distance: settings.normalized_prediction_distance,
            num_internal_stabilization_iterations: settings.num_internal_stabilization_iterations,
            ..Default::default()
        };
        if let Some(iterations) = NonZeroUsize::new(settings.num_solver_iterations) {
            integration_parameters.num_solver_iterations = iterations;
        }
        integration_parameters.num_additional_friction_iterations =
            settings.num_additional_friction_iterations;
        integration_parameters.num_internal_pgs_iterations = settings.num_internal_pgs_iterations;
        let gravity = settings.pixel_gravity;
        let liquid_gravity = settings.pixel_liquid_gravity;
        let physics_hooks = PhysicsHooksCollisionFilter {
            collision_filter_body_callback: &collision_filter_body_callback,
            collision_modify_contacts_callback: &collision_modify_contacts_callback,
            physics_collision_objects,
            last_step: RapierSpace::get_last_step(),
            ghost_collision_distance: space.get_ghost_collision_distance(),
        };
        // Initialize the event collector.
        let (collision_send, collision_recv) = crossbeam::channel::unbounded();
        let (contact_force_send, contact_force_recv) = crossbeam::channel::unbounded();
        let event_handler = ContactEventHandler::new(collision_send, contact_force_send);
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
            Some(&mut self.physics_objects.query_pipeline),
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
        for handle in self.physics_objects.island_manager.active_dynamic_bodies() {
            let active_body_info = ActiveBodyInfo {
                body_user_data: self.get_rigid_body_user_data(*handle),
            };
            space.active_body_callback(&active_body_info, physics_collision_objects);
        }
        for handle in self
            .physics_objects
            .island_manager
            .active_kinematic_bodies()
        {
            let active_body_info = ActiveBodyInfo {
                body_user_data: self.get_rigid_body_user_data(*handle),
            };
            space.active_body_callback(&active_body_info, physics_collision_objects);
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
            space.collision_event_callback(&event_info, physics_collision_objects);
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
                let send_contact_points =
                    space.contact_force_event_callback(&event_info, physics_collision_objects);
                if send_contact_points
                    && let Some(body1) = self.get_collider_rigid_body(collider1)
                    && let Some(body2) = self.get_collider_rigid_body(collider2)
                {
                    // Find the contact pair, if it exists, between two colliders
                    let mut contact_info = ContactPointInfo::default();
                    // We may also read the contact manifolds to access the contact geometry.
                    for manifold in &contact_pair.manifolds {
                        let manifold_normal = manifold.data.normal;
                        contact_info.normal = manifold_normal;
                        // Read the geometric contacts.
                        for contact_point in &manifold.points {
                            if contact_point.dist
                                > DEFAULT_EPSILON
                                    + collider1.contact_skin()
                                    + collider2.contact_skin()
                            {
                                continue;
                            }
                            let collider_pos_1 = collider1.position() * contact_point.local_p1;
                            let collider_pos_2 = collider2.position() * contact_point.local_p2;
                            let point_velocity_1 = body1.velocity_at_point(&collider_pos_1);
                            let point_velocity_2 = body2.velocity_at_point(&collider_pos_2);
                            contact_info.pixel_local_pos_1 = collider_pos_1.coords;
                            contact_info.pixel_local_pos_2 = collider_pos_2.coords;
                            contact_info.pixel_velocity_pos_1 = point_velocity_1;
                            contact_info.pixel_velocity_pos_2 = point_velocity_2;
                            contact_info.pixel_distance = contact_point.dist;
                            contact_info.pixel_impulse = contact_point.data.impulse;
                            contact_info.pixel_tangent_impulse = contact_point.data.tangent_impulse;
                            space.contact_point_callback(
                                &contact_info,
                                &event_info,
                                physics_collision_objects,
                            );
                        }
                    }
                }
            }
        }
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
        self.physics_objects.collider_set.remove(
            collider_handle,
            &mut self.physics_objects.island_manager,
            &mut self.physics_objects.rigid_body_set,
            false,
        );
    }

    pub fn get_collider_user_data(&self, collider_handle: ColliderHandle) -> UserData {
        let collider = self.physics_objects.collider_set.get(collider_handle);
        if let Some(collider) = collider {
            return UserData::new(collider.user_data);
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
        self.physics_objects.rigid_body_set.remove(
            rigid_body_handle,
            &mut self.physics_objects.island_manager,
            &mut self.physics_objects.collider_set,
            &mut self.physics_objects.impulse_joint_set,
            &mut self.physics_objects.multibody_joint_set,
            true,
        );
    }

    pub fn get_rigid_body_user_data(&self, rigid_body_handle: RigidBodyHandle) -> UserData {
        let rigid_body = self.physics_objects.rigid_body_set.get(rigid_body_handle);
        if let Some(rigid_body) = rigid_body {
            return UserData::new(rigid_body.user_data);
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
                    .get_mut(ImpulseJointHandle(handle.index));
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
    }

    pub fn get_impulse_joint(&self, handle: JointHandle) -> Option<&ImpulseJoint> {
        match handle.multibody {
            false => {
                let joint = self
                    .physics_objects
                    .impulse_joint_set
                    .get(ImpulseJointHandle(handle.index));
                joint
            }
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
    pub physics_worlds: Arena<PhysicsWorld>,
    pub shapes: Arena<SharedShape>,
}
impl PhysicsEngine {
    pub fn get_mut_world(&mut self, world_handle: WorldHandle) -> Option<&mut PhysicsWorld> {
        self.physics_worlds.get_mut(world_handle)
    }

    pub fn get_world(&self, world_handle: WorldHandle) -> Option<&PhysicsWorld> {
        self.physics_worlds.get(world_handle)
    }

    pub fn insert_shape(&mut self, shape: SharedShape) -> ShapeHandle {
        self.shapes.insert(shape)
    }

    pub fn remove_shape(&mut self, shape_handle: ShapeHandle) {
        self.shapes.remove(shape_handle);
    }

    pub fn get_shape(&self, shape_handle: ShapeHandle) -> Option<&SharedShape> {
        self.shapes.get(shape_handle)
    }

    pub fn world_create(&mut self, settings: &WorldSettings) -> WorldHandle {
        let physics_world = PhysicsWorld::new(settings);
        let world_handle = self.physics_worlds.insert(physics_world);
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.physics_objects.handle = world_handle;
        }
        world_handle
    }

    pub fn world_destroy(&mut self, world_handle: WorldHandle) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.physics_objects.handle = WorldHandle::default();
            self.physics_worlds.remove(world_handle);
        }
    }

    pub fn world_reset_if_empty(&mut self, world_handle: WorldHandle, settings: &WorldSettings) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            if physics_world.physics_objects.impulse_joint_set.is_empty()
                && physics_world
                    .physics_objects
                    .multibody_joint_set
                    .multibodies()
                    .peekable()
                    .peek()
                    .is_some()
                && physics_world.physics_objects.rigid_body_set.is_empty()
                && physics_world.physics_objects.collider_set.is_empty()
            {
                let new_physics_world = PhysicsWorld::new(settings);
                physics_world.fluids_pipeline = new_physics_world.fluids_pipeline;
                physics_world.physics_pipeline = new_physics_world.physics_pipeline;
                physics_world.physics_objects = new_physics_world.physics_objects;
            }
        }
    }

    pub fn print_stats(&mut self, world_handle: WorldHandle) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            godot_print!(
                "{} {} {}",
                physics_world.physics_objects.collider_set.len(),
                physics_world.physics_objects.rigid_body_set.len(),
                self.shapes.len(),
            );
        }
    }

    pub fn world_get_active_objects_count(&mut self, world_handle: WorldHandle) -> usize {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            return physics_world
                .physics_objects
                .island_manager
                .active_dynamic_bodies()
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

    pub fn world_step(
        &mut self,
        world_handle: WorldHandle,
        settings: &SimulationSettings,
        collision_filter_body_callback: CollisionFilterCallback,
        collision_modify_contacts_callback: CollisionModifyContactsCallback,
        space: &mut RapierSpace,
        physics_collision_objects: &mut PhysicsCollisionObjects,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world.step(
                settings,
                collision_filter_body_callback,
                collision_modify_contacts_callback,
                space,
                physics_collision_objects,
            );
        }
    }
}
