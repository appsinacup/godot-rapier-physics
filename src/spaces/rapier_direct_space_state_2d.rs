
use crate::{
    bodies::rapier_collision_object_2d::{IRapierCollisionObject2D, RapierCollisionObject2D},
    rapier2d::{
        handle::{invalid_handle, Handle}, query::{
            default_query_excluded_info, intersect_point, shape_casting, PointHitInfo, QueryExcludedInfo, RayHitInfo
        }, shape::shape_info_from_body_shape, user_data::UserData, vector::Vector
    },
    servers::rapier_physics_singleton_2d::{active_spaces_singleton, bodies_singleton, shapes_singleton, spaces_singleton},
    spaces::rapier_space_2d::RapierSpace2D,
};
use godot::{
    engine::{
        native::ObjectId, IPhysicsDirectSpaceState2DExtension, PhysicsDirectSpaceState2DExtension,
    },
    prelude::*,
};

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState2DExtension,tool)]
pub struct RapierDirectSpaceState2D {
    space: Rid,
    base: Base<PhysicsDirectSpaceState2DExtension>,
}

impl RapierDirectSpaceState2D {
    pub fn set_space(&mut self, space: Rid) {
        self.space = space;
    }
}

#[godot_api]
impl RapierDirectSpaceState2D {
    #[func]
    fn export_json(&self) -> String {
        let lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            return space.export_json();
        }
        "{}".to_string()
    }
    #[func]
    fn export_binary(&self) -> PackedByteArray {
        let lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            return space.export_binary();
        }
        PackedByteArray::default()
    }
}

pub fn is_handle_excluded_callback(
    world_handle: Handle,
    collider_handle: Handle,
    user_data: &UserData,
    handle_excluded_info: &QueryExcludedInfo,
) -> bool {
    for exclude_index in 0..handle_excluded_info.query_exclude_size {
        if handle_excluded_info.query_exclude[exclude_index] == collider_handle {
            return true;
        }
    }

    let (collision_object_2d, _) = RapierCollisionObject2D::get_collider_user_data(user_data);
    let body_lock = bodies_singleton().lock().unwrap();
    let collision_object_2d = body_lock.collision_objects.get(&collision_object_2d).unwrap();
    if handle_excluded_info.query_canvas_instance_id != collision_object_2d.get_base().get_canvas_instance_id() {
        return true;
    }

    if collision_object_2d.get_base().get_collision_layer() & handle_excluded_info.query_collision_layer_mask == 0 {
        return true;
    }

    if handle_excluded_info.query_exclude_body == collision_object_2d.get_base().get_rid().to_u64() as i64 {
        return true;
    }
    let spaces_lock = spaces_singleton().lock().unwrap();
    let active_spaces_lock = active_spaces_singleton().lock().unwrap();
    let space = active_spaces_lock.active_spaces.get(&world_handle).unwrap();
    let space = spaces_lock.spaces.get(space).unwrap();
    let direct_state = space.get_rapier_direct_state().unwrap();
    return direct_state.is_body_excluded_from_query(collision_object_2d.get_base().get_rid());
}


#[godot_api]
impl IPhysicsDirectSpaceState2DExtension for RapierDirectSpaceState2D {
    fn init(base: Base<PhysicsDirectSpaceState2DExtension>) -> Self {
        Self {
            space: Rid::Invalid,
            base,
        }
    }

    unsafe fn intersect_ray(
        &mut self,
        from: Vector2,
        to: Vector2,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        result: *mut godot::engine::native::PhysicsServer2DExtensionRayResult,
    ) -> bool {
        let mut space_handle = invalid_handle();
        {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&self.space);
            if let Some(space) = space {
                if space.locked {
                    return false;
                }
                space_handle = space.handle
            }
        }
        if !space_handle.is_valid() {
            return false;
        }

        // Raycast Info
        let begin = from;
        let end = to - from;
        let dir = end.normalized();
        let length = end.length();

        let rapier_from = Vector::new(begin.x, begin.y);
        let rapier_dir = Vector::new(dir.x, dir.y);

        let mut query_excluded_info = default_query_excluded_info();
        query_excluded_info.query_collision_layer_mask = collision_mask;

        let mut hit_info = RayHitInfo::default();
        let collide = crate::rapier2d::query::intersect_ray(
            space_handle,
            &rapier_from,
            &rapier_dir,
            length,
            collide_with_bodies,
            collide_with_areas,
            hit_from_inside,
            &mut hit_info,
            is_handle_excluded_callback,
            &query_excluded_info,
        );

        if collide {
            let result = &mut *result;
            result.position = Vector2::new(hit_info.pixel_position.x, hit_info.pixel_position.y);
            result.normal = Vector2::new(hit_info.normal.x, hit_info.normal.y);
            let (rid, shape_index) =
                RapierCollisionObject2D::get_collider_user_data(&hit_info.user_data);
            result.rid = rid;
            result.shape = shape_index as i32;
            let lock = bodies_singleton().lock().unwrap();
            let collision_object_2d = lock.collision_objects.get(&result.rid);
            if let Some(collision_object_2d) = collision_object_2d {
                let instance_id = collision_object_2d.get_base().get_instance_id();
                result.collider_id = ObjectId { id: instance_id };
                if instance_id != 0 {
                    result.collider = RapierSpace2D::_get_object_instance_hack(instance_id);
                }
            }

            return true;
        }
        false
    }

    unsafe fn intersect_point(
        &mut self,
        position: Vector2,
        canvas_instance_id: u64,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::engine::native::PhysicsServer2DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        let max_results = max_results as usize;
        if max_results <= 0 {
            return 0;
        }
        let mut space_handle = invalid_handle();
        {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&self.space);
            if let Some(space) = space {
                if space.locked {
                    return 0;
                }
                space_handle = space.handle;
            }
        }
        if !space_handle.is_valid() {
            return 0;
        }
        let rapier_pos = Vector::new(position.x, position.y);

        // Allocate memory for hit_info_array
        let mut hit_info_array: Vec<PointHitInfo> = Vec::with_capacity(max_results);
        let hit_info_ptr = hit_info_array.as_mut_ptr();

        // Initialize query_excluded_info
        let mut query_excluded_info = default_query_excluded_info();
        query_excluded_info.query_canvas_instance_id = canvas_instance_id;
        query_excluded_info.query_collision_layer_mask = collision_mask;

        // Perform intersection
        let mut result_count = intersect_point(
            space_handle,
            &rapier_pos,
            collide_with_bodies,
            collide_with_areas,
            hit_info_ptr,
            max_results,
            is_handle_excluded_callback,
            &mut query_excluded_info,
        );
        if result_count > max_results {
            result_count = max_results;
        }

        let results_slice: &mut [godot::engine::native::PhysicsServer2DExtensionShapeResult] =
            unsafe { std::slice::from_raw_parts_mut(results, max_results) };

        for i in 0..max_results {
            let hit_info = unsafe { &mut *hit_info_ptr.add(i) };

            let (rid, shape_index) =
                RapierCollisionObject2D::get_collider_user_data(&hit_info.user_data);
            results_slice[i].rid = rid;
            results_slice[i].shape = shape_index as i32;
            let lock = bodies_singleton().lock().unwrap();
            let collision_object_2d = lock.collision_objects.get(&rid);
            if let Some(collision_object_2d) = collision_object_2d {
                let instance_id = collision_object_2d.get_base().get_instance_id();
                results_slice[i].collider_id = ObjectId { id: instance_id };

                if instance_id != 0 {
                    results_slice[i].collider =
                        RapierSpace2D::_get_object_instance_hack(instance_id);
                }
            }
        }

        result_count as i32
    }

    unsafe fn intersect_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::engine::native::PhysicsServer2DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        let max_results = max_results as usize;
        let mut shape_handle = invalid_handle();
        let mut space_handle = invalid_handle();
        {
            let mut lock = shapes_singleton().lock().unwrap();
            let shape = lock.shapes.get_mut(&shape_rid);
            if let Some(shape) = shape {
                shape_handle = shape.get_rapier_shape();
            }
        }
        {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&self.space);
            if let Some(space) = space {
                space_handle = space.handle;
            }
        }
        if !shape_handle.is_valid() || !space_handle.is_valid() {
            return 0;
        }
        let rapier_motion = Vector::new(motion.x, motion.y);
        let shape_info = shape_info_from_body_shape(shape_handle, transform);

        let mut query_excluded_info = default_query_excluded_info();
        query_excluded_info.query_collision_layer_mask = collision_mask;
        let query_exclude: Vec<Handle> = Vec::with_capacity(max_results);
        query_excluded_info.query_exclude = query_exclude;
        query_excluded_info.query_exclude_size = 0;
        let mut cpt = 0;
        let results_slice: &mut [godot::engine::native::PhysicsServer2DExtensionShapeResult] =
            unsafe { std::slice::from_raw_parts_mut(results, max_results) };
        while cpt < max_results {
            let result = shape_casting(
                space_handle,
                &rapier_motion,
                shape_info.clone(),
                collide_with_bodies,
                collide_with_areas,
                is_handle_excluded_callback,
                &query_excluded_info,
                false,
            );
            if !result.collided {
                break;
            }
            query_excluded_info.query_exclude[query_excluded_info.query_exclude_size as usize] = result.collider;
            query_excluded_info.query_exclude_size+=1;
            if !result.user_data.is_valid() {
                continue;
            }
            let (rid, shape_index) =
                RapierCollisionObject2D::get_collider_user_data(&result.user_data);
            let lock = bodies_singleton().lock().unwrap();
            let collision_object_2d = lock.collision_objects.get(&rid);
            if let Some(collision_object_2d) = collision_object_2d {
                results_slice[cpt].shape = shape_index as i32;
                results_slice[cpt].rid = rid;

                let instance_id = collision_object_2d.get_base().get_instance_id();
                results_slice[cpt].collider_id = ObjectId { id: instance_id };

                if instance_id != 0 {
                    results_slice[cpt].collider =
                        RapierSpace2D::_get_object_instance_hack(instance_id);
                }
                cpt += 1;
            }
        }

        cpt as i32
    }

    unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: *mut f64,
        closest_unsafe: *mut f64,
    ) -> bool {
        let mut space_handle = invalid_handle();
        let mut shape_handle = invalid_handle();
        {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&self.space);
            if let Some(space) = space {
                if space.locked {
                    return false;
                }
                space_handle = space.handle;
            }
        }
        {
            let lock = shapes_singleton().lock().unwrap();
            let shape = lock.shapes.get(&shape_rid);
            if let Some(shape) = shape {
                shape_handle = shape.get_base().get_handle();
            }
        }
        if !space_handle.is_valid() || !shape_handle.is_valid() {
            return false;
        }

        let rapier_motion = Vector::new(motion.x, motion.y);
        let shape_info = shape_info_from_body_shape(shape_handle, transform);

        let mut query_excluded_info = default_query_excluded_info();
        query_excluded_info.query_collision_layer_mask = collision_mask;
        let hit = shape_casting(
            space_handle,
            &rapier_motion,
            shape_info,
            collide_with_bodies,
            collide_with_areas,
            is_handle_excluded_callback,
            &query_excluded_info,
            false,
        )
        .toi;
        // TODO
        *closest_safe = hit as f64;
        *closest_unsafe = hit as f64;
        true
    }

    unsafe fn collide_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut std::ffi::c_void,
        max_results: i32,
        result_count: *mut i32,
    ) -> bool {
        let mut space_handle = invalid_handle();
        let mut shape_handle = invalid_handle();
        {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&self.space);
            if let Some(space) = space {
                if space.locked {
                    return false;
                }
                space_handle = space.handle
            }
        }
        {
            let lock = shapes_singleton().lock().unwrap();
            let shape = lock.shapes.get(&shape_rid);
            if let Some(shape) = shape {
                shape_handle = shape.get_base().get_handle();
            }
        }
        if !space_handle.is_valid() || !shape_handle.is_valid() {
            return false;
        }

        let rapier_motion = Vector::new(motion.x, motion.y);

        let results_out = results as *mut Vector2;
        let shape_info = shape_info_from_body_shape(shape_handle, transform);
        let mut query_excluded_info = default_query_excluded_info();
        query_excluded_info.query_collision_layer_mask = collision_mask;
        let query_exclude: Vec<Handle> = Vec::with_capacity(max_results as usize);
        query_excluded_info.query_exclude = query_exclude;
        query_excluded_info.query_exclude_size = 0;

        let mut array_idx = 0;
        let mut cpt = 0;

        while cpt < max_results {
            let result = shape_casting(
                space_handle,
                &rapier_motion,
                shape_info.clone(),
                collide_with_bodies,
                collide_with_areas,
                is_handle_excluded_callback,
                &mut query_excluded_info,
                false,
            );
            if !result.collided {
                break;
            }
            *result_count += 1;
            query_excluded_info.query_exclude[query_excluded_info.query_exclude_size] = result.collider;
            query_excluded_info.query_exclude_size+=1;
            unsafe {
                (*results_out.add(array_idx)) =
                    Vector2::new(result.pixel_witness1.x, result.pixel_witness1.y);
                (*results_out.add(array_idx + 1)) =
                    Vector2::new(result.pixel_witness2.x, result.pixel_witness2.y);
            }
            array_idx += 2;
            cpt += 1;
        }

        array_idx > 0
    }

    unsafe fn rest_info(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: *mut godot::engine::native::PhysicsServer2DExtensionShapeRestInfo,
    ) -> bool {
        let mut space_handle = invalid_handle();
        let mut shape_handle = invalid_handle();
        {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&self.space);
            if let Some(space) = space {
                if space.locked {
                    return false;
                }
                space_handle = space.handle;
            }
        }
        {
            let lock = shapes_singleton().lock().unwrap();
            let shape = lock.shapes.get(&shape_rid);
            if let Some(shape) = shape {
                shape_handle = shape.get_base().get_handle();
            }
        }
        if !space_handle.is_valid() || !shape_handle.is_valid() {
            return false;
        }

        let rapier_motion = Vector::new(motion.x, motion.y);
        let shape_info = shape_info_from_body_shape(shape_handle, transform);

        let mut query_excluded_info = default_query_excluded_info();
        query_excluded_info.query_collision_layer_mask = collision_mask;
        let result = shape_casting(
            space_handle,
            &rapier_motion,
            shape_info,
            collide_with_bodies,
            collide_with_areas,
            is_handle_excluded_callback,
            &mut query_excluded_info,
            false,
        );
        if !result.collided {
            return false;
        }
        let (rid, shape_index) = RapierCollisionObject2D::get_collider_user_data(&result.user_data);
        let lock = bodies_singleton().lock().unwrap();
        let collision_object_2d = lock.collision_objects.get(&rid);
        let r_info = &mut *rest_info;
        if let Some(collision_object_2d) = collision_object_2d {
            let instance_id = collision_object_2d.get_base().get_instance_id();
            r_info.collider_id = ObjectId { id: instance_id };
            if let Some(body) = collision_object_2d.get_body() {
                let rel_vec = r_info.point
                    - (body.get_base().get_transform().origin + body.get_center_of_mass());
                r_info.linear_velocity = Vector2::new(
                    -body.get_angular_velocity() * rel_vec.y,
                    body.get_angular_velocity() * rel_vec.x,
                ) + body.get_linear_velocity();
            } else {
                r_info.linear_velocity = Vector2::ZERO
            }
            r_info.normal = Vector2::new(result.normal1.x, result.normal1.y);
            r_info.rid = rid;
            r_info.shape = shape_index as i32;
        }
        true
    }
}
