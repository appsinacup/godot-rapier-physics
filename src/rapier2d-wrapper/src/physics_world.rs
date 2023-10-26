use rapier2d::crossbeam;
use rapier2d::data::Arena;
use rapier2d::prelude::*;
use std::sync::Mutex;
use lazy_static::lazy_static;
use crate::handle::*;
use crate::user_data::*;
use crate::settings::*;
use crate::vector::Vector;
use crate::physics_hooks::*;

#[repr(C)]
pub struct ActiveBodyInfo {
    body_handle: Handle,
    body_user_data: UserData,
}

impl ActiveBodyInfo {
    fn new() -> ActiveBodyInfo {
        ActiveBodyInfo {
			body_handle: invalid_handle(),
			body_user_data: invalid_user_data(),
        }
    }
}


#[repr(C)]
pub struct ContactPointInfo {
    local_pos_1: Vector,
    local_pos_2: Vector,
    velocity_pos_1: Vector,
    velocity_pos_2: Vector,
    normal: Vector,
    distance: Real,
    impulse: Real,
    tangent_impulse: Real,
}

impl ContactPointInfo {
    fn new() -> ContactPointInfo {
        ContactPointInfo {
			local_pos_1: Vector { x : 0.0, y : 0.0 },
			local_pos_2: Vector { x : 0.0, y : 0.0 },
			velocity_pos_1: Vector { x : 0.0, y : 0.0 },
			velocity_pos_2: Vector { x : 0.0, y : 0.0 },
			normal: Vector { x : 0.0, y : 0.0 },
            distance: 0.0,
            impulse: 0.0,
            tangent_impulse: 0.0,
        }
    }
}

type ActiveBodyCallback = Option<extern "C" fn(world_handle : Handle, active_body_info : &ActiveBodyInfo)>;

type CollisionEventCallback = Option<extern "C" fn(world_handle : Handle, event_info : &CollisionEventInfo)>;

type ContactForceEventCallback = Option<extern "C" fn(world_handle : Handle, event_info : &ContactForceEventInfo) -> bool>;
type ContactPointCallback = Option<extern "C" fn(world_handle : Handle, contact_info : &ContactPointInfo, event_info : &ContactForceEventInfo) -> bool>;

#[repr(C)]
pub struct CollisionEventInfo {
    collider1: Handle,
    collider2: Handle,
    user_data1: UserData,
    user_data2: UserData,
	is_sensor: bool,
	is_started: bool,
	is_removed: bool,
}

impl CollisionEventInfo {
    fn new() -> CollisionEventInfo {
        CollisionEventInfo {
			collider1: invalid_handle(),
			collider2: invalid_handle(),
			user_data1: invalid_user_data(),
			user_data2: invalid_user_data(),
			is_sensor: false,
			is_started: false,
			is_removed: false,
        }
    }
}

#[repr(C)]
pub struct ContactForceEventInfo {
    collider1: Handle,
    collider2: Handle,
    user_data1: UserData,
    user_data2: UserData,
}

impl ContactForceEventInfo {
    fn new() -> ContactForceEventInfo {
        ContactForceEventInfo {
			collider1: invalid_handle(),
			collider2: invalid_handle(),
			user_data1: invalid_user_data(),
			user_data2: invalid_user_data(),
        }
    }
}

pub struct PhysicsWorld {
    pub query_pipeline: QueryPipeline,
    pub physics_pipeline : PhysicsPipeline,
    pub island_manager : IslandManager,
    pub broad_phase : BroadPhase,
    pub narrow_phase : NarrowPhase,
    pub impulse_joint_set : ImpulseJointSet,
    pub multibody_joint_set : MultibodyJointSet,
    pub ccd_solver : CCDSolver,
	
	pub sleep_linear_threshold: Real,
	pub sleep_angular_threshold: Real,
	pub sleep_time_until_sleep: Real,
    pub solver_prediction_distance : Real,
	
	pub active_body_callback : ActiveBodyCallback,
	pub collision_filter_body_callback : CollisionFilterCallback,
	pub collision_filter_sensor_callback : CollisionFilterCallback,
	pub collision_modify_contacts_callback : CollisionModifyContactsCallback,

	pub collision_event_callback : CollisionEventCallback,
	pub contact_force_event_callback : ContactForceEventCallback,

	pub contact_point_callback : ContactPointCallback,
    
    pub collider_set : ColliderSet,
    pub rigid_body_set : RigidBodySet,

	pub handle : Handle,
}

impl PhysicsWorld {
    pub fn new(settings : &WorldSettings) -> PhysicsWorld {
        PhysicsWorld {
            query_pipeline : QueryPipeline::new(),
            physics_pipeline : PhysicsPipeline::new(),
	        island_manager : IslandManager::new(),
	        broad_phase : BroadPhase::new(),
	        narrow_phase : NarrowPhase::new(),
	        impulse_joint_set : ImpulseJointSet::new(),
	        multibody_joint_set : MultibodyJointSet::new(),
	        ccd_solver : CCDSolver::new(),
    
			sleep_linear_threshold : settings.sleep_linear_threshold,
			sleep_angular_threshold : settings.sleep_angular_threshold,
			sleep_time_until_sleep : settings.sleep_time_until_sleep,
            solver_prediction_distance :settings.solver_prediction_distance,
    
			active_body_callback : None,
			collision_filter_body_callback : None,
			collision_filter_sensor_callback : None,
			collision_modify_contacts_callback: None,

			collision_event_callback : None,
			contact_force_event_callback : None,

			contact_point_callback : None,

            rigid_body_set : RigidBodySet::new(),
            collider_set : ColliderSet::new(),

			handle : invalid_handle(),
        }
    }

    pub fn step(&mut self, settings : &SimulationSettings) {
        let mut integration_parameters = IntegrationParameters::default();

        integration_parameters.dt = settings.dt;
        integration_parameters.min_ccd_dt = settings.min_ccd_dt;
        integration_parameters.erp = settings.erp;
        integration_parameters.damping_ratio = settings.damping_ratio;
        integration_parameters.joint_erp = settings.joint_erp;
        integration_parameters.joint_damping_ratio = settings.joint_damping_ratio;
        integration_parameters.allowed_linear_error = settings.allowed_linear_error;
        integration_parameters.max_penetration_correction = settings.max_penetration_correction;
        integration_parameters.prediction_distance = settings.prediction_distance;
        integration_parameters.max_velocity_iterations = settings.max_velocity_iterations;
        integration_parameters.max_velocity_friction_iterations = settings.max_velocity_friction_iterations;
        integration_parameters.max_stabilization_iterations = settings.max_stabilization_iterations;
        integration_parameters.interleave_restitution_and_friction_resolution = settings.interleave_restitution_and_friction_resolution;
        integration_parameters.min_island_size = settings.min_island_size;
        integration_parameters.max_ccd_substeps = settings.max_ccd_substeps;

        let gravity = vector![settings.gravity.x, settings.gravity.y];

		let physics_hooks = PhysicsHooksCollisionFilter {
			world_handle : self.handle,
			collision_filter_body_callback : &self.collision_filter_body_callback,
			collision_filter_sensor_callback : &self.collision_filter_sensor_callback,
			collision_modify_contacts_callback: &self.collision_modify_contacts_callback,
		};

        // Initialize the event collector.
		let (collision_send, collision_recv) = crossbeam::channel::unbounded();
		let (contact_force_send, contact_force_recv) = crossbeam::channel::unbounded();
		let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);

        self.physics_pipeline.step(
          &gravity,
          &integration_parameters,
          &mut self.island_manager,
          &mut self.broad_phase,
          &mut self.narrow_phase,
          &mut self.rigid_body_set,
          &mut self.collider_set,
          &mut self.impulse_joint_set,
          &mut self.multibody_joint_set,
          &mut self.ccd_solver,
		  Some(&mut self.query_pipeline),
          &physics_hooks,
          &event_handler,
        );
		
		if self.active_body_callback.is_some() {
			let callback = self.active_body_callback.unwrap();
			for handle in self.island_manager.active_dynamic_bodies() {
				// Send the active body event.
				let mut active_body_info = ActiveBodyInfo::new();
				active_body_info.body_handle = rigid_body_handle_to_handle(*handle);
				active_body_info.body_user_data = self.get_rigid_body_user_data(*handle);

				callback(self.handle, &active_body_info);
			}
		}
		
		if self.collision_event_callback.is_some() {
			let callback = self.collision_event_callback.unwrap();
			while let Ok(collision_event) = collision_recv.try_recv() {
				let handle1 = collision_event.collider1();
				let handle2 = collision_event.collider2();

				// Handle the collision event.
				let mut event_info = CollisionEventInfo::new();
				event_info.is_sensor = collision_event.sensor();
				event_info.is_removed = collision_event.removed();
				event_info.is_started = collision_event.started();
				event_info.collider1 = collider_handle_to_handle(handle1);
				event_info.collider2 = collider_handle_to_handle(handle2);
				event_info.user_data1 = self.get_collider_user_data(handle1);
				event_info.user_data2 = self.get_collider_user_data(handle2);

				callback(self.handle, &event_info);
			}
		}
		
		if self.contact_force_event_callback.is_some() {
			let callback = self.contact_force_event_callback.unwrap();
			while let Ok(contact_force_event) = contact_force_recv.try_recv() {
                let collider1 = self.collider_set.get(contact_force_event.collider1).unwrap();
                let collider2 = self.collider_set.get(contact_force_event.collider2).unwrap();

				// Handle the contact force event.
				let mut event_info = ContactForceEventInfo::new();
				event_info.collider1 = collider_handle_to_handle(contact_force_event.collider1);
				event_info.collider2 = collider_handle_to_handle(contact_force_event.collider2);
				event_info.user_data1 = UserData::new(collider1.user_data);
				event_info.user_data2 = UserData::new(collider2.user_data);
				
				let mut send_contact_points = callback(self.handle, &event_info);

				if send_contact_points && self.contact_point_callback.is_some() {
					let contact_callback = self.contact_point_callback.unwrap();

                    let body1: &RigidBody = self.get_collider_rigid_body(collider1).unwrap();
                    let body2: &RigidBody = self.get_collider_rigid_body(collider2).unwrap();

					// Find the contact pair, if it exists, between two colliders
					if let Some(contact_pair) = self.narrow_phase.contact_pair(contact_force_event.collider1, contact_force_event.collider2) {
						let mut contact_info = ContactPointInfo::new();

                        let mut swap = false;
                        if contact_force_event.collider1 != contact_pair.collider1 {
                            assert!(contact_force_event.collider1 == contact_pair.collider2);
                            assert!(contact_force_event.collider2 == contact_pair.collider1);
                            swap = true;
                        } else {
                            assert!(contact_force_event.collider2 == contact_pair.collider2);
                        }

						// We may also read the contact manifolds to access the contact geometry.
						for manifold in &contact_pair.manifolds {
                            let manifold_normal = manifold.data.normal;
                            contact_info.normal = Vector { x : manifold_normal.x, y : manifold_normal.y };

							// Read the geometric contacts.
							for contact_point in &manifold.points {
                                let collider_pos_1 = collider1.position() * contact_point.local_p1;
                                let collider_pos_2 = collider2.position() * contact_point.local_p2;
                                let point_velocity_1 = body1.velocity_at_point(&collider_pos_1);
                                let point_velocity_2 = body2.velocity_at_point(&collider_pos_2);

                                if swap {
                                    contact_info.local_pos_1 = Vector { x : collider_pos_2.x, y : collider_pos_2.y };
                                    contact_info.local_pos_2 = Vector { x : collider_pos_1.x, y : collider_pos_1.y };
                                    contact_info.velocity_pos_1 = Vector { x : point_velocity_2.x, y : point_velocity_2.y };
                                    contact_info.velocity_pos_2 = Vector { x : point_velocity_1.x, y : point_velocity_1.y };
                                } else {
                                    contact_info.local_pos_1 = Vector { x : collider_pos_1.x, y : collider_pos_1.y };
                                    contact_info.local_pos_2 = Vector { x : collider_pos_2.x, y : collider_pos_2.y };
                                    contact_info.velocity_pos_1 = Vector { x : point_velocity_1.x, y : point_velocity_1.y };
                                    contact_info.velocity_pos_2 = Vector { x : point_velocity_2.x, y : point_velocity_2.y };
                                }
                                contact_info.distance = contact_point.dist;
                                contact_info.impulse = contact_point.data.impulse;
                                contact_info.tangent_impulse = contact_point.data.tangent_impulse;
                                
                                send_contact_points = contact_callback(self.handle, &contact_info, &event_info);
								if !send_contact_points {
									break;
								}
                            }

							if !send_contact_points {
								break;
							}
						}
					}
				}
			}
		}
    }

    pub fn insert_collider(&mut self, collider : Collider, body_handle : Handle) -> Handle {
        if body_handle.is_valid()
        {
            let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
            let collider_handle = self.collider_set.insert_with_parent(collider, rigid_body_handle, &mut self.rigid_body_set);
            return collider_handle_to_handle(collider_handle);
        }
        else
        {
            let collider_handle = self.collider_set.insert(collider);
            return collider_handle_to_handle(collider_handle);
        }
    }

    pub fn remove_collider(&mut self, handle : Handle) {
        let collider_handle = handle_to_collider_handle(handle);
        self.collider_set.remove(collider_handle
            , &mut self.island_manager
            , &mut self.rigid_body_set
            , false
        );
    }

	pub fn get_collider_user_data(&self, collider_handle : ColliderHandle) -> UserData {
		let collider = self.collider_set.get(collider_handle);
		if !collider.is_some() {
			return invalid_user_data();
		} else{
			return UserData::new(collider.unwrap().user_data);
		}
	}

	pub fn get_collider_rigid_body(&self, collider : &Collider) -> Option<&RigidBody> {
		let parent = collider.parent();
        if parent.is_some() {
            return self.rigid_body_set.get(parent.unwrap());
        } else {
            return None;
        }
	}
    
    pub fn insert_rigid_body(&mut self, rigid_body : RigidBody) -> Handle {
        let body_handle = self.rigid_body_set.insert(rigid_body);
        return rigid_body_handle_to_handle(body_handle);
    }
    
    pub fn remove_rigid_body(&mut self, body_handle : Handle) {
        let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
        self.rigid_body_set.remove(rigid_body_handle
            , &mut self.island_manager
            , &mut self.collider_set
            , &mut self.impulse_joint_set
            , &mut self.multibody_joint_set
            , true
        );
    }

	pub fn get_rigid_body_user_data(&self, rigid_body_handle : RigidBodyHandle) -> UserData {
		let rigid_body = self.rigid_body_set.get(rigid_body_handle);
		if !rigid_body.is_some() {
			return invalid_user_data();
		} else{
			return UserData::new(rigid_body.unwrap().user_data);
		}
	}

	pub fn insert_joint(&mut self, body_handle_1 : Handle, body_handle_2 : Handle, joint : impl Into<GenericJoint>) -> Handle {
		let rigid_body_1_handle = handle_to_rigid_body_handle(body_handle_1);
		let rigid_body_2_handle = handle_to_rigid_body_handle(body_handle_2);
        
		let joint_handle = self.impulse_joint_set.insert(rigid_body_1_handle, rigid_body_2_handle, joint, true);
		return joint_handle_to_handle(joint_handle);
	}

	pub fn remove_joint(&mut self, handle : Handle) {
        let joint_handle = handle_to_joint_handle(handle);
		self.impulse_joint_set.remove(joint_handle, true);
	}
}

pub struct PhysicsEngine {
	pub physics_worlds : Arena<PhysicsWorld>,
	pub shapes : Arena<SharedShape>,
}

impl PhysicsEngine {
    fn new() -> PhysicsEngine {
        PhysicsEngine {
			physics_worlds: Arena::new(),
			shapes: Arena::new(),
        }
    }

	pub fn insert_world(&mut self, world : PhysicsWorld) -> Handle {
		let world_handle = self.physics_worlds.insert(world);
		return world_handle_to_handle(world_handle);
	}

	pub fn remove_world(&mut self, handle : Handle) {
        let world_handle = handle_to_world_handle(handle);
		self.physics_worlds.remove(world_handle);
	}

	pub fn get_world(&mut self, handle : Handle) -> &mut PhysicsWorld {
        let world_handle = handle_to_world_handle(handle);
		let world = self.physics_worlds.get_mut(world_handle);
		assert!(world.is_some());
		return world.unwrap();
	}

	pub fn insert_shape(&mut self, shape : SharedShape) -> Handle {
		let shape_handle = self.shapes.insert(shape);
		return shape_handle_to_handle(shape_handle);
	}

	pub fn remove_shape(&mut self, handle : Handle) {
        let shape_handle = handle_to_shape_handle(handle);
		self.shapes.remove(shape_handle);
	}

	pub fn get_shape(&mut self, handle : Handle) -> &SharedShape {
        let shape_handle = handle_to_shape_handle(handle);
		let shape = self.shapes.get(shape_handle);
		assert!(shape.is_some());
		return shape.unwrap();
	}
}

lazy_static! {
	pub static ref SINGLETON: Mutex<PhysicsEngine> = Mutex::new(PhysicsEngine::new());
}

#[no_mangle]
pub extern "C" fn world_create(settings: &WorldSettings) -> Handle {
	let physics_world = PhysicsWorld::new(settings);
	let mut physics_engine = SINGLETON.lock().unwrap();
	let world_handle = physics_engine.insert_world(physics_world);
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.handle = world_handle;
	return world_handle;
}

#[no_mangle]
pub extern "C" fn world_destroy(world_handle : Handle) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.handle = invalid_handle();
	physics_engine.remove_world(world_handle);
}

#[no_mangle]
pub extern "C" fn world_step(world_handle : Handle, settings : &SimulationSettings) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.step(settings);
}

#[no_mangle]
pub extern "C" fn world_set_active_body_callback(world_handle : Handle, callback : ActiveBodyCallback) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world: &mut PhysicsWorld = physics_engine.get_world(world_handle);
	physics_world.active_body_callback = callback;
}

#[no_mangle]
pub extern "C" fn world_set_body_collision_filter_callback(world_handle : Handle, callback : CollisionFilterCallback) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.collision_filter_body_callback = callback;
}

#[no_mangle]
pub extern "C" fn world_set_sensor_collision_filter_callback(world_handle : Handle, callback : CollisionFilterCallback) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.collision_filter_sensor_callback = callback;
}

#[no_mangle]
pub extern "C" fn world_set_modify_contacts_callback(world_handle : Handle, callback : CollisionModifyContactsCallback) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.collision_modify_contacts_callback = callback;
}

#[no_mangle]
pub extern "C" fn world_set_collision_event_callback(world_handle : Handle, callback : CollisionEventCallback) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.collision_event_callback = callback;
}

#[no_mangle]
pub extern "C" fn world_set_contact_force_event_callback(world_handle : Handle, callback : ContactForceEventCallback) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.contact_force_event_callback = callback;
}

#[no_mangle]
pub extern "C" fn world_set_contact_point_callback(world_handle : Handle, callback : ContactPointCallback) {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
	physics_world.contact_point_callback = callback;
}
