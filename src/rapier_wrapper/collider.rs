use godot::builtin::math::FloatExt;
use godot::global::godot_error;
use rapier::prelude::*;
use salva::integrations::rapier::ColliderSampling;
use salva::object::Boundary;
use salva::parry::either::Either::Left;
use salva::parry::either::Either::Right;

use crate::rapier_wrapper::prelude::*;
const SUBDIVISIONS: u32 = 20;
#[cfg(feature = "dim2")]
fn skew_polyline(vertices: &Vec<Point<Real>>, skew: Real) -> SharedShape {
    // Apply skew transformation to the vertices
    let mut skewed_vertices = Vec::new();
    for vertex in vertices {
        let mut skewed_vertex = *vertex;
        skewed_vertex.x -= skewed_vertex.y * skew;
        skewed_vertices.push(skewed_vertex);
    }
    let len = vertices.len();
    let mut indices = vec![];
    for i in 0..len {
        indices.push([i as u32, ((i + 1) % len) as u32]);
    }
    let collider = ColliderBuilder::convex_decomposition(&skewed_vertices, &indices);
    collider.shape
}
// Function to skew a shape
#[cfg(feature = "dim2")]
pub fn skew_shape(shape: &SharedShape, shape_info: ShapeInfo) -> SharedShape {
    use godot::builtin::math::FloatExt;
    let skew = shape_info.skew;
    if skew.is_zero_approx() {
        return shape.clone();
    }
    match shape.shape_type() {
        ShapeType::Compound => {
            if let Some(compound) = shape.as_compound() {
                let shapes = compound.shapes();
                let mut transformed_shapes = Vec::new();
                for (position, sub_shape) in shapes.iter() {
                    let skewed_sub_shape = skew_shape(sub_shape, shape_info);
                    let transformed_position = *position;
                    transformed_shapes.push((transformed_position, skewed_sub_shape));
                }
                return SharedShape::compound(transformed_shapes);
            }
        }
        ShapeType::Ball => {
            if let Some(ball) = shape.as_ball() {
                return skew_polyline(&ball.to_polyline(SUBDIVISIONS), skew);
            }
        }
        ShapeType::Cuboid => {
            if let Some(cuboid) = shape.as_cuboid() {
                return skew_polyline(&cuboid.to_polyline(), skew);
            }
        }
        ShapeType::Polyline => {
            if let Some(polyline) = shape.as_polyline() {
                return skew_polyline(&polyline.vertices().to_vec(), skew);
            }
        }
        ShapeType::ConvexPolygon => {
            if let Some(convex_polygon) = shape.as_convex_polygon() {
                return skew_polyline(&convex_polygon.points().to_vec(), skew);
            }
        }
        ShapeType::Capsule => {
            if let Some(capsule) = shape.as_capsule() {
                return skew_polyline(&capsule.to_polyline(SUBDIVISIONS), skew);
            }
        }
        _ => {
            godot_error!("Shape type not supported for skewing");
        }
    }
    shape.clone()
}
#[cfg(feature = "dim3")]
fn skew_shape(shape: &SharedShape, _shape_info: ShapeInfo) -> SharedShape {
    shape.clone()
}
pub fn scale_shape(shape: &SharedShape, shape_info: ShapeInfo) -> SharedShape {
    let shape = skew_shape(&shape.clone(), shape_info);
    let scale = shape_info.scale;
    if (scale - Vector::repeat(1.0))
        .norm_squared()
        .is_zero_approx()
    {
        return shape.clone();
    }
    match shape.shape_type() {
        ShapeType::Ball => {
            if let Some(new_shape) = shape.as_ball() {
                if let Some(new_shape) = new_shape.scaled(&scale.abs(), SUBDIVISIONS) {
                    match new_shape {
                        Left(shape) => return SharedShape::new(shape),
                        Right(shape) => return SharedShape::new(shape),
                    }
                }
            }
        }
        ShapeType::Cuboid => {
            if let Some(new_shape) = shape.as_cuboid() {
                return SharedShape::new(new_shape.scaled(&scale.abs()));
            }
        }
        ShapeType::HalfSpace => {
            if let Some(new_shape) = shape.as_halfspace() {
                if let Some(new_shape) = new_shape.scaled(&scale.abs()) {
                    return SharedShape::new(new_shape);
                }
            }
        }
        ShapeType::Polyline => {
            if let Some(new_shape) = shape.as_polyline() {
                return SharedShape::new(new_shape.clone().scaled(&scale));
            }
        }
        #[cfg(feature = "dim3")]
        ShapeType::TriMesh => {
            if let Some(new_shape) = shape.as_trimesh() {
                return SharedShape::new(new_shape.clone().scaled(&scale));
            }
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cylinder => {
            if let Some(new_shape) = shape.as_cylinder() {
                if let Some(new_shape) = new_shape.scaled(&scale, SUBDIVISIONS) {
                    match new_shape {
                        Left(shape) => return SharedShape::new(shape),
                        Right(shape) => return SharedShape::new(shape),
                    }
                }
            }
        }
        #[cfg(feature = "dim2")]
        ShapeType::ConvexPolygon => {
            if let Some(new_shape) = shape.as_convex_polygon() {
                if let Some(new_shape) = new_shape.clone().scaled(&scale) {
                    return SharedShape::new(new_shape);
                }
            }
        }
        #[cfg(feature = "dim3")]
        ShapeType::ConvexPolyhedron => {
            if let Some(new_shape) = shape.as_convex_polyhedron() {
                if let Some(new_shape) = new_shape.clone().scaled(&scale) {
                    return SharedShape::new(new_shape);
                }
            }
        }
        #[cfg(feature = "dim3")]
        ShapeType::HeightField => {
            if let Some(new_shape) = shape.as_heightfield() {
                let new_shape = new_shape.clone().scaled(&scale);
                return SharedShape::new(new_shape);
            }
        }
        ShapeType::Capsule => {
            if let Some(new_shape) = shape.as_capsule() {
                if let Some(new_shape) = new_shape.scaled(&scale, SUBDIVISIONS) {
                    match new_shape {
                        Left(shape) => return SharedShape::new(shape),
                        Right(shape) => return SharedShape::new(shape),
                    }
                }
            }
        }
        ShapeType::Compound => {
            if let Some(new_shape) = shape.as_compound() {
                let new_shapes = new_shape.shapes();
                let mut shapes_vec = Vec::new();
                for shape in new_shapes {
                    let new_shape = scale_shape(&shape.1, shape_info);
                    shapes_vec.push((shape.0, new_shape));
                }
                return SharedShape::compound(shapes_vec);
            }
        }
        _ => {
            godot_error!("Shape type not supported for scaling");
        }
    }
    shape.clone()
}
pub struct Material {
    pub friction: Real,
    pub restitution: Real,
    pub contact_skin: Real,
    pub collision_mask: u32,
    pub collision_layer: u32,
}
impl Material {
    pub fn new(collision_layer: u32, collision_mask: u32) -> Material {
        Material {
            friction: 0.0,
            restitution: 0.0,
            contact_skin: 0.0,
            collision_layer,
            collision_mask,
        }
    }
}
fn shape_is_halfspace(shape: &SharedShape) -> bool {
    if shape.shape_type() == ShapeType::Compound {
        if let Some(shape) = shape.as_compound() {
            for shape in shape.shapes() {
                if shape_is_halfspace(&shape.1) {
                    return true;
                }
            }
        }
    }
    shape.shape_type() == ShapeType::HalfSpace
}
impl PhysicsEngine {
    pub fn collider_set_modify_contacts_enabled(
        &mut self,
        world_handle: WorldHandle,
        collider_handle: ColliderHandle,
        enable: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            if let Some(collider) = physics_world
                .physics_objects
                .collider_set
                .get_mut(collider_handle)
            {
                let mut active_events = collider.active_hooks();
                if enable {
                    active_events |= ActiveHooks::MODIFY_SOLVER_CONTACTS;
                } else {
                    active_events &= !ActiveHooks::MODIFY_SOLVER_CONTACTS;
                }
                collider.set_active_hooks(active_events);
            }
        }
    }

    pub fn collider_set_filter_contacts_enabled(
        &mut self,
        world_handle: WorldHandle,
        collider_handle: ColliderHandle,
        enable: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            if let Some(collider) = physics_world
                .physics_objects
                .collider_set
                .get_mut(collider_handle)
            {
                let mut active_events = collider.active_hooks();
                if enable {
                    active_events |= ActiveHooks::FILTER_CONTACT_PAIRS;
                } else {
                    active_events &= !ActiveHooks::FILTER_CONTACT_PAIRS;
                }
                collider.set_active_hooks(active_events);
            }
        }
    }

    pub fn collider_create_solid(
        &mut self,
        world_handle: WorldHandle,
        shape_handle: ShapeHandle,
        mat: &Material,
        body_handle: RigidBodyHandle,
        user_data: &UserData,
    ) -> ColliderHandle {
        if let Some(shape) = self.get_shape(shape_handle) {
            let is_shape_halfspace = shape_is_halfspace(shape);
            let mut collider = ColliderBuilder::new(shape.clone())
                .contact_force_event_threshold(-Real::MAX)
                .density(1.0)
                .build();
            collider.set_friction(mat.friction);
            collider.set_restitution(mat.restitution);
            collider.set_friction_combine_rule(CoefficientCombineRule::Min);
            collider.set_restitution_combine_rule(CoefficientCombineRule::Sum);
            collider.set_collision_groups(InteractionGroups {
                memberships: Group::from(mat.collision_layer),
                filter: Group::from(mat.collision_mask),
            });
            collider.set_solver_groups(InteractionGroups {
                memberships: Group::GROUP_1,
                filter: Group::GROUP_1,
            });
            collider.set_contact_skin(mat.contact_skin);
            collider.set_contact_force_event_threshold(-Real::MAX);
            collider.user_data = user_data.get_data();
            if let Some(physics_world) = self.get_mut_world(world_handle) {
                let collider_handle = physics_world.insert_collider(collider, body_handle);
                // register fluid coupling. Dynamic coupling doens't work for halfspace
                if !is_shape_halfspace {
                    let boundary_handle = physics_world
                        .fluids_pipeline
                        .liquid_world
                        .add_boundary(Boundary::new(Vec::new()));
                    physics_world.fluids_pipeline.coupling.register_coupling(
                        boundary_handle,
                        collider_handle,
                        ColliderSampling::DynamicContactSampling,
                    );
                }
                return collider_handle;
            }
        }
        ColliderHandle::invalid()
    }

    pub fn collider_set_user_data(
        &mut self,
        world_handle: WorldHandle,
        collider_handle: ColliderHandle,
        user_data: &UserData,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            if let Some(collider) = physics_world
                .physics_objects
                .collider_set
                .get_mut(collider_handle)
            {
                collider.user_data = user_data.get_data();
            }
        }
    }

    pub fn collider_create_sensor(
        &mut self,
        world_handle: WorldHandle,
        shape_handle: ShapeHandle,
        mat: &Material,
        body_handle: RigidBodyHandle,
        user_data: &UserData,
    ) -> ColliderHandle {
        if let Some(shape) = self.get_shape(shape_handle) {
            let mut collider = ColliderBuilder::new(shape.clone()).build();
            collider.set_sensor(true);
            collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
            // less data to serialize
            collider.set_collision_groups(InteractionGroups {
                memberships: Group::from(mat.collision_layer),
                filter: Group::from(mat.collision_mask),
            });
            collider.set_solver_groups(InteractionGroups {
                memberships: Group::GROUP_1,
                filter: Group::GROUP_1,
            });
            let mut collision_types = collider.active_collision_types();
            // Area vs Area
            collision_types |= ActiveCollisionTypes::FIXED_FIXED;
            // Area vs CharacterBody
            collision_types |= ActiveCollisionTypes::KINEMATIC_FIXED;
            collider.set_active_collision_types(collision_types);
            collider.user_data = user_data.get_data();
            collider.set_active_hooks(ActiveHooks::FILTER_INTERSECTION_PAIR);
            if let Some(physics_world) = self.get_mut_world(world_handle) {
                return physics_world.insert_collider(collider, body_handle);
            }
        }
        ColliderHandle::invalid()
    }

    pub fn collider_destroy(&mut self, world_handle: WorldHandle, collider_handle: ColliderHandle) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            physics_world
                .fluids_pipeline
                .coupling
                .unregister_coupling(collider_handle);
            physics_world.remove_collider(collider_handle);
        }
    }

    pub fn collider_set_transform(
        &mut self,
        world_handle: WorldHandle,
        collider_handle: ColliderHandle,
        shape_info: ShapeInfo,
    ) {
        if let Some(shape) = self.get_shape(shape_info.handle) {
            let new_shape = scale_shape(shape, shape_info);
            if let Some(physics_world) = self.get_mut_world(world_handle) {
                if let Some(collider) = physics_world
                    .physics_objects
                    .collider_set
                    .get_mut(collider_handle)
                {
                    collider.set_position_wrt_parent(shape_info.transform);
                    collider.set_shape(new_shape);
                }
            }
        }
    }

    pub fn collider_set_contact_force_events_enabled(
        &mut self,
        world_handle: WorldHandle,
        collider_handle: ColliderHandle,
        enable: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            if let Some(collider) = physics_world
                .physics_objects
                .collider_set
                .get_mut(collider_handle)
            {
                let mut active_events = collider.active_events();
                if enable {
                    active_events |= ActiveEvents::CONTACT_FORCE_EVENTS;
                } else {
                    active_events &= !ActiveEvents::CONTACT_FORCE_EVENTS;
                }
                collider.set_active_events(active_events);
            }
        }
    }
}
