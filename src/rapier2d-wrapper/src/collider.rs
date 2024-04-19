use rapier2d::na::Point2;
use rapier2d::na::Vector2;
use rapier2d::prelude::*;
use salva2d::integrations::rapier::ColliderSampling;
use salva2d::object::Boundary;
use crate::handle::*;
use crate::shape::ShapeInfo;
use crate::user_data::*;
use crate::vector::Vector;
use crate::physics_world::*;
use crate::convert::*;

const SUBDIVISIONS: u32 = 50;

fn skew_polyline(vertices: &Vec<Point2<Real>>, skew: Real) -> SharedShape {
    // Apply skew transformation to the vertices
    let mut skewed_vertices = Vec::new();
    for vertex in vertices {
        let mut skewed_vertex = vertex.clone();
        skewed_vertex.x -= skewed_vertex.y * skew;
        skewed_vertices.push(skewed_vertex);
    }

    if let Some(convex_polyline) = SharedShape::convex_hull(&skewed_vertices.clone()) {
        return convex_polyline;
    }

    SharedShape::polyline(skewed_vertices, None)
}

// Function to skew a shape
pub fn skew_shape(shape: &SharedShape, skew: Real) -> SharedShape {
    if skew == 0.0 {
        return shape.clone();
    }
    match shape.shape_type() {
        ShapeType::Compound => {
            let compound = shape.as_compound().unwrap();
            let shapes = compound.shapes();
            let mut transformed_shapes = Vec::new();
            for (position, sub_shape) in shapes.iter() {
                let skewed_sub_shape = skew_shape(sub_shape, skew);
                let transformed_position = *position;
                transformed_shapes.push((transformed_position, skewed_sub_shape));
            }
            return SharedShape::compound(transformed_shapes);
        }
        ShapeType::Ball => {
            let ball_polyline = shape.as_ball().unwrap().to_polyline(SUBDIVISIONS);
            return skew_polyline(&ball_polyline, skew);
        }
        ShapeType::Cuboid => {
            let cuboid_polyline = shape.as_cuboid().unwrap().to_polyline();
            return skew_polyline(&cuboid_polyline, skew);
        }
        ShapeType::Polyline => {
            let polyline = shape.as_polyline().unwrap();
            return skew_polyline(&polyline.vertices().to_vec(), skew);
        }
        ShapeType::ConvexPolygon => {
            let convex_polygon = shape.as_convex_polygon().unwrap();
            let pooints =convex_polygon.points();
            return skew_polyline(&pooints.to_vec(), skew);
        }
        ShapeType::Capsule => {
            let capsule = shape.as_capsule().unwrap().to_polyline(SUBDIVISIONS);
            return skew_polyline(&capsule, skew);
        }
        _ => {
            return shape.clone();
        }
    }
}



pub fn scale_shape(shape: &SharedShape, scale: &Vector2<Real>) -> SharedShape {
    if scale.x == 1.0 && scale.y == 1.0 {
        return shape.clone();
    }
    let shape_type = shape.shape_type();
    if shape_type == ShapeType::Ball {
        let new_shape = shape.as_ball().unwrap().scaled(scale, SUBDIVISIONS).unwrap();
        if new_shape.is_left() {
            let shape = new_shape.unwrap_left();
            return SharedShape::new(shape);
        } else {
            let shape = new_shape.unwrap_right();
            return SharedShape::new(shape);
        }
    }
    else if shape_type == ShapeType::Cuboid {
        let new_shape = shape.as_cuboid().unwrap().scaled(scale);
        return SharedShape::new(new_shape);
    }
    else if shape_type == ShapeType::HalfSpace {
        let new_shape = shape.as_halfspace().unwrap().scaled(scale).unwrap();
        return SharedShape::new(new_shape);
    }
    else if shape_type == ShapeType::Polyline {
        let new_shape = shape.as_polyline().unwrap().clone().scaled(scale);
        return SharedShape::new(new_shape);
    }
    else if shape_type == ShapeType::ConvexPolygon {
        let new_shape = shape.as_convex_polygon().unwrap().clone().scaled(scale).unwrap();
        return SharedShape::new(new_shape);
    }
    else if shape_type == ShapeType::Compound {
        let new_shapes = shape.as_compound().unwrap().shapes();
        let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::new();
        for shape in new_shapes {
            let new_shape = scale_shape(&shape.1, scale);
            shapes_vec.push((shape.0, new_shape));
        }
        return SharedShape::compound(shapes_vec);
    }
    return shape.clone();
}

#[repr(C)]
pub struct Material {
    pub friction : Real,
    pub restitution : Real,
}

#[no_mangle]
pub extern "C" fn default_material() -> Material {
    Material {
        friction : 1.0,
        restitution : 0.0,
    }
}

#[no_mangle]
pub extern "C" fn collider_create_solid(world_handle : Handle, shape_handle : Handle, mat : &Material, body_handle : Handle, user_data : &UserData) -> Handle {
    let mut physics_engine = singleton().lock().unwrap();
    let shape = physics_engine.get_shape(shape_handle);
    let mut collider = ColliderBuilder::new(shape.clone()).contact_force_event_threshold(-Real::MAX).build();
    // TODO update when https://github.com/dimforge/rapier/issues/622 is fixed
    if mat.friction >= 0.0 {
        collider.set_friction(mat.friction);
    }
    if mat.restitution >= 0.0 {
        collider.set_restitution(mat.restitution);
    }
    collider.set_friction_combine_rule(CoefficientCombineRule::Multiply);
    collider.set_restitution_combine_rule(CoefficientCombineRule::Max);
    collider.set_density(0.0);
    collider.set_contact_force_event_threshold(-Real::MAX);
    collider.user_data = user_data.get_data();
    collider.set_active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS);
    let physics_world = physics_engine.get_world(world_handle);
    let handle = physics_world.insert_collider(collider, body_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let boundary_handle = physics_world.fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new()));
    physics_world.fluids_pipeline.coupling.register_coupling(
        boundary_handle,
        collider_handle,
        ColliderSampling::DynamicContactSampling,
    );
    return handle
}

#[no_mangle]
pub extern "C" fn collider_create_sensor(world_handle : Handle, shape_handle : Handle, body_handle : Handle, user_data : &UserData) -> Handle {
    let mut physics_engine = singleton().lock().unwrap();
    let shape = physics_engine.get_shape(shape_handle);
    let mut collider = ColliderBuilder::new(shape.clone()).build();
    collider.set_sensor(true);
    collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
    let mut collision_types = collider.active_collision_types();
    // Area vs Area
    collision_types |= ActiveCollisionTypes::FIXED_FIXED;
    // Area vs CharacterBody
    collision_types |= ActiveCollisionTypes::KINEMATIC_FIXED;
    collider.set_active_collision_types(collision_types);
    collider.user_data = user_data.get_data();
    collider.set_active_hooks(ActiveHooks::FILTER_INTERSECTION_PAIR);
    let physics_world = physics_engine.get_world(world_handle);
    return physics_world.insert_collider(collider, body_handle);
}

#[no_mangle]
pub extern "C" fn collider_destroy(world_handle : Handle, handle : Handle) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    physics_world.fluids_pipeline.coupling.unregister_coupling(
        collider_handle
    );
    return physics_world.remove_collider(handle);
}

#[no_mangle]
pub extern "C" fn collider_get_position(world_handle : Handle, handle : Handle) -> Vector {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get(collider_handle);
    assert!(collider.is_some());
    let collider_vector = collider.unwrap().translation();
    return vector_meters_to_pixels(&Vector { x : collider_vector.x, y : collider_vector.y });
}

#[no_mangle]
pub extern "C" fn collider_get_angle(world_handle : Handle, handle : Handle) -> Real {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get(collider_handle);
    assert!(collider.is_some());
    return collider.unwrap().rotation().angle();
}

#[no_mangle]
pub extern "C" fn collider_set_transform(world_handle : Handle, handle : Handle, shape_info: ShapeInfo) {
    {   
        let position = &vector_pixels_to_meters(&shape_info.pixel_position);

        let mut physics_engine = singleton().lock().unwrap();
        let physics_world = physics_engine.get_world(world_handle);
        let collider_handle = handle_to_collider_handle(handle);
        let collider = physics_world.collider_set.get_mut(collider_handle);
        assert!(collider.is_some());
        let collider = collider.unwrap();
        collider.set_position_wrt_parent(Isometry::new(vector![position.x, position.y], shape_info.rotation));
    }
    {
        let mut physics_engine = singleton().lock().unwrap();
        let shape = physics_engine.get_shape(shape_info.handle);
        let skewed_shape = skew_shape(shape, shape_info.skew);
        let new_shape = scale_shape(&skewed_shape, &Vector2::<Real>::new(shape_info.scale.x, shape_info.scale.y));
        let physics_world = physics_engine.get_world(world_handle);
        let collider_handle = handle_to_collider_handle(handle);
        let collider = physics_world.collider_set.get_mut(collider_handle);
        assert!(collider.is_some());
        collider.unwrap().set_shape(new_shape);
    }
}

#[no_mangle]
pub extern "C" fn collider_set_collision_events_enabled(world_handle : Handle, handle : Handle, enable : bool) {
    let mut physics_engine = singleton().lock().unwrap();
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
    let mut physics_engine = singleton().lock().unwrap();
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
