use rapier2d::crossbeam;
use rapier2d::data::Arena;
use rapier2d::data::Index;
use rapier2d::parry;
use rapier2d::prelude::*;

// Handle

#[repr(C)]
#[derive(Copy, Clone, Eq, Hash, PartialEq)]
pub struct Handle {
    id : u32,
    generation : u32,
}

impl Default for Handle {
	fn default() -> Handle {
        Handle {
			id : u32::MAX,
			generation : u32::MAX,
        }
    }
}

impl Handle {
    pub fn is_valid(&self) -> bool {
        return (self.id != u32::MAX) && (self.generation != u32::MAX);
    }
}

#[no_mangle]
pub extern "C" fn invalid_handle() -> Handle {
    Handle {
        id : u32::MAX,
        generation : u32::MAX,
    }
}

#[no_mangle]
pub extern "C" fn is_handle_valid(handle : Handle) -> bool {
    return handle.is_valid();
}

#[no_mangle]
pub extern "C" fn are_handles_equal(handle1 : Handle, handle2 : Handle) -> bool {
    return (handle1.id == handle2.id) && (handle1.generation == handle2.generation);
}

// User data

#[repr(C)]
#[derive(Copy, Clone, Eq, Hash, PartialEq)]
pub struct UserData {
    part1 : u64,
    part2 : u64,
}

impl UserData {
    fn new(data : u128) -> UserData {
		let data2 : u128 = data >> 64;
		let data1 : u128 = data - (data2 << 64);
        UserData {
			part1 : data1.try_into().unwrap(),
			part2 : data2.try_into().unwrap(),
        }
    }

    pub fn is_valid(&self) -> bool {
        return (self.part1 != u64::MAX) && (self.part2 != u64::MAX);
    }

    pub fn get_data(&self) -> u128 {
		let data1 : u128 = self.part1.into();
		let data2 : u128 = self.part2.into();
        return data1 + (data2 << 64);
    }
}

#[no_mangle]
pub extern "C" fn invalid_user_data() -> UserData {
    UserData {
        part1 : u64::MAX,
        part2 : u64::MAX,
    }
}

#[no_mangle]
pub extern "C" fn is_user_data_valid(user_data : UserData) -> bool {
    return user_data.is_valid();
}

// Active body info

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

// Collision filter info

#[repr(C)]
pub struct CollisionFilterInfo {
    user_data1: UserData,
    user_data2: UserData,
}

impl CollisionFilterInfo {
    fn new() -> CollisionFilterInfo {
        CollisionFilterInfo {
			user_data1: invalid_user_data(),
			user_data2: invalid_user_data(),
        }
    }
}

// Events info

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

// Contact point info

#[repr(C)]
pub struct ContactPointInfo {
    local_pos_1: Vector,
    local_pos_2: Vector,
    velocity_pos_1: Vector,
    velocity_pos_2: Vector,
    normal: Vector,
    distance: f32,
    impulse: f32,
    tangent_impulse: f32,
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

// Query results

#[repr(C)]
pub struct RayHitInfo {
    position: Vector,
    normal: Vector,
    collider: Handle,
    user_data: UserData,
}

#[repr(C)]
pub struct PointHitInfo {
    collider: Handle,
    user_data: UserData,
}

#[repr(C)]
pub struct ShapeCastResult {
    collided: bool,
    toi : f32,
    witness1 : Vector,
    witness2 : Vector,
    normal1 : Vector,
    normal2 : Vector,
    collider: Handle,
    user_data: UserData
}

impl ShapeCastResult {
    fn new() -> ShapeCastResult {
        ShapeCastResult {
            collided : false,
            toi : 1.0, 
			collider : invalid_handle(),
            witness1 : Vector{ x: 0.0, y: 0.0 },
            witness2 : Vector{ x: 0.0, y: 0.0 },
            normal1 : Vector{ x: 0.0, y: 0.0 },
            normal2 : Vector{ x: 0.0, y: 0.0 },
            user_data: UserData { part1: 0, part2: 0 }
        }
    }
}

#[repr(C)]
pub struct ContactResult {
    collided: bool,
    distance: f32,
    point1 : Vector,
    point2 : Vector,
    normal1 : Vector,
    normal2 : Vector
}

impl ContactResult {
    fn new() -> ContactResult {
        ContactResult {
            collided : false,
            distance : 0.0, 
            point1 : Vector{ x: 0.0, y: 0.0 },
            point2 : Vector{ x: 0.0, y: 0.0 },
            normal1 : Vector{ x: 0.0, y: 0.0 },
            normal2 : Vector{ x: 0.0, y: 0.0 }
        }
    }
}

// Shapes

#[repr(C)]
pub struct ShapeInfo {
    handle: Handle,
    position : Vector,
    rotation : f32,
}

// Simulation Settings

#[repr(C)]
pub struct SimulationSettings {
    delta_time : f32,
    max_velocity_iterations : usize,
    max_velocity_friction_iterations : usize,
    max_stabilization_iterations : usize,
    gravity : Vector,
}

#[no_mangle]
pub extern "C" fn default_simulation_settings() -> SimulationSettings {
    SimulationSettings {
        delta_time : 1.0 / 60.0,
        max_velocity_iterations : 4,
        max_velocity_friction_iterations : 8,
        max_stabilization_iterations : 1,
        gravity : Vector { x : 0.0, y : -9.81 },
    }
}

// World Settings

#[repr(C)]
pub struct WorldSettings {
	sleep_linear_threshold: f32,
	sleep_angular_threshold: f32,
	sleep_time_until_sleep: f32,
    solver_damping_ratio : f32,
    solver_prediction_distance : f32,
}

#[no_mangle]
pub extern "C" fn default_world_settings() -> WorldSettings {
    WorldSettings {
		sleep_linear_threshold : 0.1,
		sleep_angular_threshold : 0.1,
		sleep_time_until_sleep : 1.0,
        solver_damping_ratio : 0.42,
        solver_prediction_distance : 0.002,
    }
}

// Material

#[repr(C)]
pub struct Material {
    friction : f32,
    restitution : f32,
}

#[no_mangle]
pub extern "C" fn default_material() -> Material {
    Material {
        friction : 0.5,
        restitution : 0.0,
    }
}

#[repr(C)]
pub struct Vector {
    x : f32,
    y : f32,
}

// Misc Utilities

fn point_array_to_vec(data : &Vector, data_count : usize) -> Vec::<Point::<Real>> {
    let mut vec = Vec::<Point::<Real>>::with_capacity(data_count);
    unsafe {
        let data_raw = std::slice::from_raw_parts(data, data_count);
        for point in data_raw {
            vec.push(Point::<Real> { coords : vector![point.x, point.y] });
        }
    }
    return vec;
}

fn world_handle_to_handle(world_handle : Index) -> Handle {
    let raw_parts = world_handle.into_raw_parts();
    return Handle {
        id : raw_parts.0,
        generation : raw_parts.1,
    }
}


fn handle_to_world_handle(handle : Handle) -> Index {
    return Index::from_raw_parts(handle.id, handle.generation);
}

fn shape_handle_to_handle(shape_handle : Index) -> Handle {
    let raw_parts = shape_handle.into_raw_parts();
    return Handle {
        id : raw_parts.0,
        generation : raw_parts.1,
    }
}


fn handle_to_shape_handle(handle : Handle) -> Index {
    return Index::from_raw_parts(handle.id, handle.generation);
}

fn collider_handle_to_handle(collider_handle : ColliderHandle) -> Handle {
    let raw_parts = collider_handle.into_raw_parts();
    return Handle {
        id : raw_parts.0,
        generation : raw_parts.1,
    }
}

fn handle_to_collider_handle(handle : Handle) -> ColliderHandle {
    return ColliderHandle::from_raw_parts(handle.id, handle.generation);
}

fn rigid_body_handle_to_handle(rigid_body_handle : RigidBodyHandle) -> Handle {
    let raw_parts = rigid_body_handle.into_raw_parts();
    return Handle {
        id : raw_parts.0,
        generation : raw_parts.1,
    }
}

fn handle_to_rigid_body_handle(handle : Handle) -> RigidBodyHandle {
    return RigidBodyHandle::from_raw_parts(handle.id, handle.generation);
}

fn joint_handle_to_handle(joint_handle : ImpulseJointHandle) -> Handle {
    let raw_parts = joint_handle.into_raw_parts();
    return Handle {
        id : raw_parts.0,
        generation : raw_parts.1,
    }
}

fn handle_to_joint_handle(handle : Handle) -> ImpulseJointHandle {
    return ImpulseJointHandle::from_raw_parts(handle.id, handle.generation);
}

// Physics Engine

struct PhysicsEngine {
	physics_worlds : Arena<PhysicsWorld>,
	shapes : Arena<SharedShape>,
}

impl PhysicsEngine {
    fn new() -> PhysicsEngine {
        PhysicsEngine {
			physics_worlds: Arena::new(),
			shapes: Arena::new(),
        }
    }

	fn insert_world(&mut self, world : PhysicsWorld) -> Handle {
		let world_handle = self.physics_worlds.insert(world);
		return world_handle_to_handle(world_handle);
	}

	fn remove_world(&mut self, handle : Handle) {
        let world_handle = handle_to_world_handle(handle);
		self.physics_worlds.remove(world_handle);
	}

	fn get_world(&mut self, handle : Handle) -> &mut PhysicsWorld {
        let world_handle = handle_to_world_handle(handle);
		let world = self.physics_worlds.get_mut(world_handle);
		assert!(world.is_some());
		return world.unwrap();
	}

	fn insert_shape(&mut self, shape : SharedShape) -> Handle {
		let shape_handle = self.shapes.insert(shape);
		return shape_handle_to_handle(shape_handle);
	}

	fn remove_shape(&mut self, handle : Handle) {
        let shape_handle = handle_to_shape_handle(handle);
		self.shapes.remove(shape_handle);
	}

	fn get_shape(&mut self, handle : Handle) -> &SharedShape {
        let shape_handle = handle_to_shape_handle(handle);
		let shape = self.shapes.get(shape_handle);
		assert!(shape.is_some());
		return shape.unwrap();
	}
}

#[macro_use]
extern crate lazy_static;
use std::sync::Mutex;

lazy_static! {
    static ref SINGLETON: Mutex<PhysicsEngine> = Mutex::new(PhysicsEngine::new());
}

// Physics World

type ActiveBodyCallback = Option<extern "C" fn(world_handle : Handle, active_body_info : &ActiveBodyInfo)>;
type CollisionFilterCallback = Option<extern "C" fn(world_handle : Handle, filter_info : &CollisionFilterInfo) -> bool>;

type CollisionEventCallback = Option<extern "C" fn(world_handle : Handle, event_info : &CollisionEventInfo)>;

type ContactForceEventCallback = Option<extern "C" fn(world_handle : Handle, event_info : &ContactForceEventInfo) -> bool>;
type ContactPointCallback = Option<extern "C" fn(world_handle : Handle, contact_info : &ContactPointInfo) -> bool>;

struct PhysicsHooksCollisionFilter<'a> {
	world_handle : Handle,
	collision_filter_body_callback : &'a CollisionFilterCallback,
	collision_filter_sensor_callback : &'a CollisionFilterCallback,
}

impl<'a> PhysicsHooks for PhysicsHooksCollisionFilter<'a> {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
		if self.collision_filter_body_callback.is_some() {
			let callback = self.collision_filter_body_callback.unwrap();

			let user_data1 = context.colliders[context.collider1].user_data;
			let user_data2 = context.colliders[context.collider2].user_data;
			
			let mut filter_info = CollisionFilterInfo::new();
			filter_info.user_data1 = UserData::new(user_data1);
			filter_info.user_data2 = UserData::new(user_data2);
			
			// Handle contact filtering for rigid bodies
			if !callback(self.world_handle, &filter_info) {
				return None;
			}
		}
		
		return Some(SolverFlags::COMPUTE_IMPULSES);
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        if self.collision_filter_sensor_callback.is_some() {
			let callback = self.collision_filter_sensor_callback.unwrap();

			let user_data1 = context.colliders[context.collider1].user_data;
			let user_data2 = context.colliders[context.collider2].user_data;

			let mut filter_info = CollisionFilterInfo::new();
			filter_info.user_data1 = UserData::new(user_data1);
			filter_info.user_data2 = UserData::new(user_data2);
			
			// Handle intersection filtering for sensors
			return callback(self.world_handle, &filter_info);
		}

		return true;
    }
}

struct PhysicsWorld {
    query_pipeline: QueryPipeline,
    physics_pipeline : PhysicsPipeline,
    island_manager : IslandManager,
    broad_phase : BroadPhase,
    narrow_phase : NarrowPhase,
    impulse_joint_set : ImpulseJointSet,
    multibody_joint_set : MultibodyJointSet,
    ccd_solver : CCDSolver,
	
	sleep_linear_threshold: f32,
	sleep_angular_threshold: f32,
	sleep_time_until_sleep: f32,
    solver_damping_ratio : f32,
    solver_prediction_distance : f32,
	
	active_body_callback : ActiveBodyCallback,
	collision_filter_body_callback : CollisionFilterCallback,
	collision_filter_sensor_callback : CollisionFilterCallback,

	collision_event_callback : CollisionEventCallback,
	contact_force_event_callback : ContactForceEventCallback,

	contact_point_callback : ContactPointCallback,
    
    collider_set : ColliderSet,
    rigid_body_set : RigidBodySet,

	handle : Handle,
}

impl PhysicsWorld {
    fn new(settings : &WorldSettings) -> PhysicsWorld {
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
            solver_damping_ratio : settings.solver_damping_ratio,
            solver_prediction_distance :settings.solver_prediction_distance,
    
			active_body_callback : None,
			collision_filter_body_callback : None,
			collision_filter_sensor_callback : None,

			collision_event_callback : None,
			contact_force_event_callback : None,

			contact_point_callback : None,

            rigid_body_set : RigidBodySet::new(),
            collider_set : ColliderSet::new(),

			handle : invalid_handle(),
        }
    }

    fn step(&mut self, settings : &SimulationSettings) {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = settings.delta_time;
        integration_parameters.max_velocity_iterations = settings.max_velocity_iterations;
        integration_parameters.max_velocity_friction_iterations = settings.max_velocity_friction_iterations;
        integration_parameters.max_stabilization_iterations = settings.max_stabilization_iterations;
		integration_parameters.damping_ratio = self.solver_damping_ratio;
		integration_parameters.prediction_distance = self.solver_prediction_distance;

        let gravity = vector![settings.gravity.x, settings.gravity.y];

		let physics_hooks = PhysicsHooksCollisionFilter {
			world_handle : self.handle,
			collision_filter_body_callback : &self.collision_filter_body_callback,
			collision_filter_sensor_callback : &self.collision_filter_sensor_callback,
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
                                
                                send_contact_points = contact_callback(self.handle, &contact_info);
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

    fn insert_collider(&mut self, collider : Collider, body_handle : Handle) -> Handle {
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

    fn remove_collider(&mut self, handle : Handle) {
        let collider_handle = handle_to_collider_handle(handle);
        self.collider_set.remove(collider_handle
            , &mut self.island_manager
            , &mut self.rigid_body_set
            , false
        );
    }

	fn get_collider_user_data(&self, collider_handle : ColliderHandle) -> UserData {
		let collider = self.collider_set.get(collider_handle);
		if !collider.is_some() {
			return invalid_user_data();
		} else{
			return UserData::new(collider.unwrap().user_data);
		}
	}

	fn get_collider_rigid_body(&self, collider : &Collider) -> Option<&RigidBody> {
		let parent = collider.parent();
        if parent.is_some() {
            return self.rigid_body_set.get(parent.unwrap());
        } else {
            return None;
        }
	}
    
    fn insert_rigid_body(&mut self, rigid_body : RigidBody) -> Handle {
        let body_handle = self.rigid_body_set.insert(rigid_body);
        return rigid_body_handle_to_handle(body_handle);
    }
    
    fn remove_rigid_body(&mut self, body_handle : Handle) {
        let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
        self.rigid_body_set.remove(rigid_body_handle
            , &mut self.island_manager
            , &mut self.collider_set
            , &mut self.impulse_joint_set
            , &mut self.multibody_joint_set
            , true
        );
    }

	fn get_rigid_body_user_data(&self, rigid_body_handle : RigidBodyHandle) -> UserData {
		let rigid_body = self.rigid_body_set.get(rigid_body_handle);
		if !rigid_body.is_some() {
			return invalid_user_data();
		} else{
			return UserData::new(rigid_body.unwrap().user_data);
		}
	}

	fn insert_joint(&mut self, body_handle_1 : Handle, body_handle_2 : Handle, joint : impl Into<GenericJoint>) -> Handle {
		let rigid_body_1_handle = handle_to_rigid_body_handle(body_handle_1);
		let rigid_body_2_handle = handle_to_rigid_body_handle(body_handle_2);

		let joint_handle = self.impulse_joint_set.insert(rigid_body_1_handle, rigid_body_2_handle, joint, true);
		return joint_handle_to_handle(joint_handle);
	}

	fn remove_joint(&mut self, handle : Handle) {
        let joint_handle = handle_to_joint_handle(handle);
		self.impulse_joint_set.remove(joint_handle, true);
	}
}

// World interface

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
	let physics_world = physics_engine.get_world(world_handle);
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

// Shape interface

#[no_mangle]
pub extern "C" fn shape_create_box(size : &Vector) -> Handle {
	let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_halfspace(normal : &Vector) -> Handle {
	let shape = SharedShape::halfspace(UnitVector::new_normalize(vector![normal.x, normal.y]));
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_circle(radius : f32) -> Handle {
	let shape = SharedShape::ball(radius);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_capsule(half_height : f32, radius : f32) -> Handle {
	let shape = SharedShape::capsule_y(half_height, radius);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_convex_polyline(points : &Vector, point_count : usize) -> Handle {
    let points_vec = point_array_to_vec(points, point_count);
    let shape_data = SharedShape::convex_polyline(points_vec);
	if shape_data.is_none() {
		return Handle::default();
	}
    let shape = shape_data.unwrap();
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_convave_polyline(points : &Vector, point_count : usize) -> Handle {
    let points_vec = point_array_to_vec(points, point_count);
    let shape = SharedShape::polyline(points_vec, None);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_compound(shapes : &ShapeInfo, shape_count : usize) -> Handle {
    let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::with_capacity(shape_count);
    let mut physics_engine = SINGLETON.lock().unwrap();
    unsafe {
        let data_raw = std::slice::from_raw_parts(shapes, shape_count);
        for shape_info in data_raw {
            let shape = physics_engine.get_shape(shape_info.handle);
            let pos = vector![shape_info.position.x, shape_info.position.y];
            shapes_vec.push((Isometry::new(pos, shape_info.rotation), shape.clone()));
        }
    }

    let shape = SharedShape::compound(shapes_vec);
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_destroy(shape_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.remove_shape(shape_handle);
}

// Collider interface

#[no_mangle]
pub extern "C" fn collider_create_solid(world_handle : Handle, shape_handle : Handle, mat : &Material, body_handle : Handle, user_data : &UserData) -> Handle {
	let mut physics_engine = SINGLETON.lock().unwrap();
    let shape = physics_engine.get_shape(shape_handle);
	let mut collider = ColliderBuilder::new(shape.clone()).build();
    collider.set_friction(mat.friction);
    collider.set_restitution(mat.restitution);
    collider.set_friction_combine_rule(CoefficientCombineRule::Multiply);
    collider.set_restitution_combine_rule(CoefficientCombineRule::Max);
    collider.set_density(0.0);
	collider.user_data = user_data.get_data();
	collider.set_active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS);
	let physics_world = physics_engine.get_world(world_handle);
    return physics_world.insert_collider(collider, body_handle);
}

#[no_mangle]
pub extern "C" fn collider_create_sensor(world_handle : Handle, shape_handle : Handle, body_handle : Handle, user_data : &UserData) -> Handle {
	let mut physics_engine = SINGLETON.lock().unwrap();
    let shape = physics_engine.get_shape(shape_handle);
	let mut collider = ColliderBuilder::new(shape.clone()).build();
    collider.set_sensor(true);
	collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
	let mut collision_types = collider.active_collision_types();
	collision_types |= ActiveCollisionTypes::FIXED_FIXED;
	collider.set_active_collision_types(collision_types);
	collider.user_data = user_data.get_data();
	collider.set_active_hooks(ActiveHooks::FILTER_INTERSECTION_PAIR);
	let physics_world = physics_engine.get_world(world_handle);
    return physics_world.insert_collider(collider, body_handle);
}

#[no_mangle]
pub extern "C" fn collider_destroy(world_handle : Handle, handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    return physics_world.remove_collider(handle);
}

#[no_mangle]
pub extern "C" fn collider_get_position(world_handle : Handle, handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get(collider_handle);
    assert!(collider.is_some());
    let collider_vector = collider.unwrap().translation();
    return Vector { x : collider_vector.x, y : collider_vector.y };
}

#[no_mangle]
pub extern "C" fn collider_get_angle(world_handle : Handle, handle : Handle) -> f32 {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get(collider_handle);
    assert!(collider.is_some());
    return collider.unwrap().rotation().angle();
}

#[no_mangle]
pub extern "C" fn collider_set_transform(world_handle : Handle, handle : Handle, pos : &Vector, rot : f32) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get_mut(collider_handle);
    assert!(collider.is_some());
    collider.unwrap().set_position_wrt_parent(Isometry::new(vector![pos.x, pos.y], rot));
}

#[no_mangle]
pub extern "C" fn collider_set_collision_events_enabled(world_handle : Handle, handle : Handle, enable : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get_mut(collider_handle);
    assert!(collider.is_some());
	let collider_access = collider.unwrap();
	let mut active_events = collider_access.active_events();
	if enable {
		active_events |= ActiveEvents::COLLISION_EVENTS;
	} else {
		active_events &= !ActiveEvents::COLLISION_EVENTS;
	}
	collider_access.set_active_events(active_events);
}

#[no_mangle]
pub extern "C" fn collider_set_contact_force_events_enabled(world_handle : Handle, handle : Handle, enable : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get_mut(collider_handle);
    assert!(collider.is_some());
	let collider_access = collider.unwrap();
	let mut active_events = collider_access.active_events();
	if enable {
		active_events |= ActiveEvents::CONTACT_FORCE_EVENTS;
	} else {
		active_events &= !ActiveEvents::CONTACT_FORCE_EVENTS;
	}
	collider_access.set_active_events(active_events);
}

// Rigid body interface

fn set_rigid_body_properties_internal(rigid_body : &mut RigidBody, pos : &Vector, rot : f32) {
    rigid_body.set_rotation(Rotation::new(rot), false);
    rigid_body.set_translation(vector![pos.x, pos.y], false);
}

#[no_mangle]
pub extern "C" fn body_create_fixed(world_handle : Handle, pos : &Vector, rot : f32, user_data : &UserData) -> Handle {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let mut rigid_body = RigidBodyBuilder::fixed().build();
    set_rigid_body_properties_internal(&mut rigid_body, pos, rot);
	rigid_body.user_data = user_data.get_data();
    let body_handle = physics_world.rigid_body_set.insert(rigid_body);
    return rigid_body_handle_to_handle(body_handle);
}

#[no_mangle]
pub extern "C" fn body_create_dynamic(world_handle : Handle, pos : &Vector, rot : f32, user_data : &UserData) -> Handle {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let mut rigid_body = RigidBodyBuilder::dynamic().can_sleep(true).sleeping(true).build();
	let activation = rigid_body.activation_mut();
	// TODO: set parameter in Rapier once added, not possible for now
	//activation.time_since_can_sleep = physics_world.sleep_time_until_sleep;
    activation.linear_threshold = physics_world.sleep_linear_threshold;
    activation.angular_threshold = physics_world.sleep_angular_threshold;
    set_rigid_body_properties_internal(&mut rigid_body, pos, rot);
	rigid_body.user_data = user_data.get_data();
    return physics_world.insert_rigid_body(rigid_body);
}

#[no_mangle]
pub extern "C" fn body_destroy(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    physics_world.remove_rigid_body(body_handle);
}

#[no_mangle]
pub extern "C" fn body_get_position(world_handle : Handle, body_handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    let body_vector = body.unwrap().translation();
    return Vector { x : body_vector.x, y : body_vector.y };
}

#[no_mangle]
pub extern "C" fn body_get_angle(world_handle : Handle, body_handle : Handle) -> f32 {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    return body.unwrap().rotation().angle();
}

#[no_mangle]
pub extern "C" fn body_set_transform(world_handle : Handle, body_handle : Handle, pos : &Vector, rot : f32, wake_up : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_position(Isometry::new(vector![pos.x, pos.y], rot), wake_up);
}

#[no_mangle]
pub extern "C" fn body_get_linear_velocity(world_handle : Handle, body_handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    let body_vel = body.unwrap().linvel();
    return Vector { x : body_vel.x, y : body_vel.y };
}

#[no_mangle]
pub extern "C" fn body_set_linear_velocity(world_handle : Handle, body_handle : Handle, vel : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_linvel(vector![vel.x, vel.y], true);
}

#[no_mangle]
pub extern "C" fn body_update_material(world_handle : Handle, body_handle : Handle, mat : &Material) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    for collider in body.unwrap().colliders() {
        if let Some(col) = physics_world.collider_set.get_mut(*collider) {
            col.set_friction(mat.friction);
            col.set_restitution(mat.restitution)
        }
    }
}

#[no_mangle]
pub extern "C" fn body_get_angular_velocity(world_handle : Handle, body_handle : Handle) -> f32 {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    return body.unwrap().angvel();
}

#[no_mangle]
pub extern "C" fn body_set_angular_velocity(world_handle : Handle, body_handle : Handle, vel : f32) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_angvel(vel, true);
}

#[no_mangle]
pub extern "C" fn body_set_linear_damping(world_handle : Handle, body_handle : Handle, linear_damping : f32) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_linear_damping(linear_damping);
}

#[no_mangle]
pub extern "C" fn body_set_angular_damping(world_handle : Handle, body_handle : Handle, angular_damping : f32) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_angular_damping(angular_damping);
}

#[no_mangle]
pub extern "C" fn body_set_gravity_scale(world_handle : Handle, body_handle : Handle, gravity_scale : f32, wake_up : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_gravity_scale(gravity_scale, wake_up);
}

#[no_mangle]
pub extern "C" fn body_set_can_sleep(world_handle : Handle, body_handle : Handle, can_sleep:  bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let body = body.unwrap();
    
    if can_sleep && body.activation().angular_threshold == -1.0 {
		let activation = body.activation_mut();
        activation.angular_threshold = physics_world.sleep_angular_threshold;
        activation.linear_threshold = physics_world.sleep_linear_threshold;
    } else if !can_sleep && body.activation().angular_threshold != -1.0 {
		let activation = body.activation_mut();
        activation.angular_threshold = -1.0;
        activation.linear_threshold = -1.0;
    }

    // TODO: Check if is requiered
    if !can_sleep && body.is_sleeping() {
        body.wake_up(true);
    }
}

#[no_mangle]
pub extern "C" fn body_is_ccd_enabled(world_handle : Handle, body_handle : Handle) -> bool {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().is_ccd_enabled()
}

#[no_mangle]
pub extern "C" fn body_set_ccd_enabled(world_handle : Handle, body_handle : Handle, enable: bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().enable_ccd(enable);
}

#[no_mangle]
pub extern "C" fn body_set_mass_properties(world_handle : Handle, body_handle : Handle, mass : f32, inertia : f32, local_com : &Vector, wake_up : bool, force_update : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
	let body_ref = body.unwrap();
	body_ref.set_additional_mass_properties(MassProperties::new(point![local_com.x, local_com.y], mass, inertia), wake_up);
    if force_update {
        body_ref.recompute_mass_properties_from_colliders(&physics_world.collider_set);
    }
}

#[no_mangle]
pub extern "C" fn body_add_force(world_handle : Handle, body_handle : Handle, force : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().add_force(vector!(force.x, force.y), true);
}

#[no_mangle]
pub extern "C" fn body_add_force_at_point(world_handle : Handle, body_handle : Handle, force : &Vector, point : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().add_force_at_point(vector!(force.x, force.y),point![point.x, point.y] , true);
}

#[no_mangle]
pub extern "C" fn body_add_torque(world_handle : Handle, body_handle : Handle, torque: f32) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().add_torque(torque, true);
}

#[no_mangle]
pub extern "C" fn body_apply_impulse(world_handle : Handle, body_handle : Handle, impulse : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().apply_impulse(vector!(impulse.x, impulse.y), true);
}

#[no_mangle]
pub extern "C" fn body_apply_impulse_at_point(world_handle : Handle, body_handle : Handle, impulse : &Vector, point : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().apply_impulse_at_point(vector!(impulse.x, impulse.y),point![point.x, point.y] , true);
}

#[no_mangle]
pub extern "C" fn body_get_constant_force(world_handle : Handle, body_handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let constant_force = body.unwrap().user_force();
    return Vector { x : constant_force.x, y : constant_force.y };
}

#[no_mangle]
pub extern "C" fn body_get_constant_torque(world_handle : Handle, body_handle : Handle) -> f32 {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().user_torque()
}

#[no_mangle]
pub extern "C" fn body_apply_torque_impulse(world_handle : Handle, body_handle : Handle, torque_impulse : f32) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().apply_torque_impulse(torque_impulse, true);
}

#[no_mangle]
pub extern "C" fn body_reset_torques(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().reset_torques(false);
}

#[no_mangle]
pub extern "C" fn body_reset_forces(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().reset_forces(false);
}

#[no_mangle]
pub extern "C" fn body_wake_up(world_handle : Handle, body_handle : Handle, strong : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    // TODO: check if this is OK, at the startup call to RapierPhysicsServer2D::free where body is not some
    assert!(body.is_some());
    let body = body.unwrap();
    if body.is_sleeping() {
        body.wake_up(strong);
    }
}

#[no_mangle]
pub extern "C" fn body_force_sleep(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().sleep();
}

// Joint interface

#[no_mangle]
pub extern "C" fn joint_create_revolute(world_handle : Handle, body_handle_1 : Handle, body_handle_2 : Handle, anchor_1 : &Vector, anchor_2 : &Vector) -> Handle {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = RevoluteJointBuilder::new()
    .local_anchor1(point!(anchor_1.x, anchor_1.y))
    .local_anchor2(point!(anchor_2.x, anchor_2.y));
    
	return physics_world.insert_joint(body_handle_1, body_handle_2, joint);
}

#[no_mangle]
pub extern "C" fn joint_create_prismatic(world_handle : Handle, body_handle_1 : Handle, body_handle_2 : Handle, axis : &Vector, anchor_1 : &Vector, anchor_2 : &Vector, limits : &Vector) -> Handle {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = PrismaticJointBuilder::new(UnitVector::new_normalize(vector![axis.x, axis.y]))
    .local_anchor1(point!(anchor_1.x, anchor_1.y))
    .local_anchor2(point!(anchor_2.x, anchor_2.y))
	.limits([limits.x, limits.y]);
    
	return physics_world.insert_joint(body_handle_1, body_handle_2, joint);
}

#[no_mangle]
pub extern "C" fn joint_destroy(world_handle : Handle, joint_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

	return physics_world.remove_joint(joint_handle);
}

// Scene queries

// PhysicsDirectSpaceState2DExtension
type QueryHandleExcludedCallback = Option<extern "C" fn(world_handle : Handle, collider_handle : Handle, user_data : &UserData) -> bool>;

#[no_mangle]
pub extern "C" fn intersect_ray(world_handle : Handle, from : &Vector, dir : &Vector, length: f32, collide_with_body: bool, collide_with_area: bool, hit_from_inside: bool, hit_info : &mut RayHitInfo, handle_excluded_callback: QueryHandleExcludedCallback) -> bool {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let ray = Ray::new(point![from.x, from.y], vector![dir.x, dir.y]);
    let solid = true;
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle));
        }
        return true;
    };

    filter.predicate = Some(&predicate);
     
    let mut result = false;
    physics_world.query_pipeline.intersections_with_ray(&physics_world.rigid_body_set, &physics_world.collider_set, &ray, length, solid, filter,
        |handle, intersection| {
            // Callback called on each collider hit by the ray.

            if hit_from_inside || intersection.toi != 0.0 {
                result = true;

                let hit_point = ray.point_at(intersection.toi);
                let hit_normal = intersection.normal;
                hit_info.position = Vector {
                    x: hit_point.x,
                    y: hit_point.y,
                };
                hit_info.normal = Vector {
                    x: hit_normal.x,
                    y: hit_normal.y,
                };
                hit_info.collider = collider_handle_to_handle(handle);
				hit_info.user_data = physics_world.get_collider_user_data(handle);
                return false; // We found a collision hit.
            }
            true // Continue to search.
        },
    );

    return result;
}

#[no_mangle]
pub extern "C" fn intersect_point(world_handle : Handle, position : &Vector,  collide_with_body: bool, collide_with_area: bool, hit_info_array : *mut PointHitInfo, hit_info_length : usize, handle_excluded_callback: QueryHandleExcludedCallback) -> usize {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let point = Point::new(position.x, position.y);
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle));
        }
        return true;
    };

    filter.predicate = Some(&predicate);
  
    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(hit_info_array, hit_info_length));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.intersections_with_point( &physics_world.rigid_body_set, &physics_world.collider_set, &point, filter,
        |handle| {
        // Callback called on each collider hit by the ray.
        hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
        hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(handle);
        cpt_hit += 1;
        let keep_searching = cpt_hit < hit_info_length;
        keep_searching // Continue to search collisions if we still have space for results.
    });

    return cpt_hit;
}

#[no_mangle]
pub extern "C" fn shape_casting(world_handle : Handle, motion : &Vector, position: &Vector, rotation: f32, shape_handle : Handle, collide_with_body: bool, collide_with_area: bool, handle_excluded_callback: QueryHandleExcludedCallback) -> ShapeCastResult {
    let mut physics_engine = SINGLETON.lock().unwrap();

    let shared_shape = physics_engine.get_shape(shape_handle).clone();

	let physics_world = physics_engine.get_world(world_handle);
    
    let shape_vel = vector![motion.x, motion.y];
    let shape_transform = Isometry::new(vector![position.x, position.y], rotation);
    
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle));
        }
        return true;
    };

    filter.predicate = Some(&predicate);

    let mut result = ShapeCastResult::new();
    
    if let Some((collider_handle, hit)) = physics_world.query_pipeline.cast_shape(
        &physics_world.rigid_body_set, &physics_world.collider_set, &shape_transform, &shape_vel, shared_shape.as_ref(), 1.0, false, filter
    ) {
        result.collided = true;
        result.toi = hit.toi;
        result.witness1 = Vector{ x: hit.witness1.x, y: hit.witness1.y };
        result.witness2 = Vector{ x: hit.witness2.x, y: hit.witness2.y };
        result.normal1 = Vector{ x: hit.normal1.x, y: hit.normal1.y };
        result.normal2 = Vector{ x: hit.normal2.x, y: hit.normal2.y };
        result.collider = collider_handle_to_handle(collider_handle);
        result.user_data = physics_world.get_collider_user_data(collider_handle);
        return result;
    }
    return result;
}

#[no_mangle]
pub extern "C" fn intersect_shape(world_handle : Handle, position: &Vector, rotation: f32, shape_handle : Handle, collide_with_body: bool, collide_with_area: bool, hit_info_array : *mut PointHitInfo, hit_info_length : usize, handle_excluded_callback: QueryHandleExcludedCallback) -> usize {
    let mut physics_engine = SINGLETON.lock().unwrap();

    let shared_shape = physics_engine.get_shape(shape_handle).clone();

	let physics_world = physics_engine.get_world(world_handle);
    
    let shape_transform = Isometry::new(vector![position.x, position.y], rotation);
    
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle));
        }
        return true;
    };

    filter.predicate = Some(&predicate);

    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(hit_info_array, hit_info_length));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.intersections_with_shape (
        &physics_world.rigid_body_set, &physics_world.collider_set, &shape_transform, shared_shape.as_ref(), filter, 
        |handle| {
            // Callback called on each collider hit by the ray.
            hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
            hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(handle);
            cpt_hit += 1;
            let keep_searching = cpt_hit < hit_info_length;
            keep_searching // Continue to search collisions if we still have space for results.
        }
    );

    return cpt_hit;
}

#[no_mangle]
pub extern "C" fn intersect_aabb(world_handle : Handle, aabb_min : &Vector, aabb_max : &Vector, collide_with_body: bool, collide_with_area: bool, hit_info_array : *mut PointHitInfo, hit_info_length : usize, handle_excluded_callback: QueryHandleExcludedCallback) -> usize {
    let mut physics_engine = SINGLETON.lock().unwrap();

	let physics_world = physics_engine.get_world(world_handle);
    
    // let aabb_transform = Isometry::new(vector![position.x, position.y], rotation);
    let aabb_min_point = Point::new(aabb_min.x, aabb_min.y);
    let aabb_max_point = Point::new(aabb_max.x, aabb_max.y);
    
    // let transformed_aabb_min = aabb_transform * aabb_min_point;
    // let transformed_aabb_max = aabb_transform * aabb_max_point;
    
    let aabb = Aabb {
        mins: aabb_min_point,
        maxs: aabb_max_point,
    };

    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(hit_info_array, hit_info_length));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.colliders_with_aabb_intersecting_aabb(&aabb,         
        |handle| {

        let mut valid_hit = false;
        if let Some(collider) = physics_world.collider_set.get(*handle) {

            // type filder
            if collider.is_sensor() && collide_with_area {
                valid_hit = true;
            } else if !collider.is_sensor() && collide_with_body {
                valid_hit = true;
            }

            if valid_hit && handle_excluded_callback.is_some(){
                valid_hit = !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(*handle), &physics_world.get_collider_user_data(*handle));
            }
        }
        
        if !valid_hit {
            return true; // continue
        }

        // Callback called on each collider hit by the ray.
        hit_info_slice[cpt_hit].collider = collider_handle_to_handle(*handle);
        hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(*handle);
        cpt_hit += 1;
        let keep_searching = cpt_hit < hit_info_length;
        keep_searching // Continue to search collisions if we still have space for results.
    });

    return cpt_hit;
}

#[no_mangle]
pub extern "C" fn shapes_contact(world_handle : Handle, shape_handle1 : Handle, position1: &Vector, rotation1: f32, shape_handle2 : Handle, position2: &Vector, rotation2: f32, margin: f32) -> ContactResult {
    let mut physics_engine = SINGLETON.lock().unwrap();

	let physics_world = physics_engine.get_world(world_handle);

    let prediction = f32::max(physics_world.solver_prediction_distance, margin);

    let shared_shape1 = physics_engine.get_shape(shape_handle1).clone();
    let shared_shape2 = physics_engine.get_shape(shape_handle2).clone();

    let shape_transform1 = Isometry::new(vector![position1.x, position1.y], rotation1);
    let shape_transform2 = Isometry::new(vector![position2.x, position2.y], rotation2);
    
    let mut result = ContactResult::new();
    
    if let Ok(Some(contact)) = parry::query::contact(
        &shape_transform1, shared_shape1.as_ref(), &shape_transform2, shared_shape2.as_ref(), prediction
    ) {
        result.collided = true;
        result.distance = contact.dist;
        result.point1 = Vector{ x: contact.point1.x, y: contact.point1.y };
        result.point2 = Vector{ x: contact.point2.x, y: contact.point2.y };
        result.normal1 = Vector{ x: contact.normal1.x, y: contact.normal1.y };
        result.normal2 = Vector{ x: contact.normal2.x, y: contact.normal2.y };
        return result;
    }
    return result;
}
