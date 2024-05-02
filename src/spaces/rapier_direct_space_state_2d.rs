use crate::{
    bodies::rapier_collision_object_2d::{IRapierCollisionObject2D, RapierCollisionObject2D},
    rapier2d::{
        handle::{invalid_handle, Handle},
        query::{
            default_query_excluded_info, intersect_point, shape_casting, PointHitInfo, RayHitInfo,
        },
        shape::shape_info_from_body_shape,
        vector::Vector,
    },
    servers::rapier_physics_singleton_2d::physics_singleton,
    spaces::rapier_space_2d::RapierSpace2D,
};
use godot::{
    engine::{
        native::ObjectId, IPhysicsDirectSpaceState2DExtension, PhysicsDirectSpaceState2DExtension,
    },
    prelude::*,
};

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState2DExtension)]
pub struct RapierDirectSpaceState2D {
    space: Rid,
    base: Base<PhysicsDirectSpaceState2DExtension>,
}

#[godot_api]
impl RapierDirectSpaceState2D {
    #[func]
    pub fn set_space(&mut self, space: Rid) {
        self.space = space;
    }
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
        let lock = physics_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            if space.locked || !space.handle.is_valid() {
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
                space.handle,
                &rapier_from,
                &rapier_dir,
                length,
                collide_with_bodies,
                collide_with_areas,
                hit_from_inside,
                &mut hit_info,
                RapierSpace2D::_is_handle_excluded_callback,
                &query_excluded_info,
            );

            if collide {
                let result = &mut *result;
                result.position =
                    Vector2::new(hit_info.pixel_position.x, hit_info.pixel_position.y);
                result.normal = Vector2::new(hit_info.normal.x, hit_info.normal.y);
                let (rid, shape_index) =
                    RapierCollisionObject2D::get_collider_user_data(&hit_info.user_data);
                result.rid = rid;
                result.shape = shape_index as i32;
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
        let lock = physics_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            if space.locked || !space.handle.is_valid() || max_results < 0 {
                return 0;
            }

            let rapier_pos = Vector::new(position.x, position.y);

            // Allocate memory for hit_info_array
            let mut hit_info_array: Vec<PointHitInfo> = Vec::with_capacity(max_results as usize);
            let hit_info_ptr = hit_info_array.as_mut_ptr();

            // Initialize query_excluded_info
            let mut query_excluded_info = default_query_excluded_info();
            query_excluded_info.query_canvas_instance_id = canvas_instance_id;
            query_excluded_info.query_collision_layer_mask = collision_mask;

            // Perform intersection
            let mut result_count = intersect_point(
                space.handle,
                &rapier_pos,
                collide_with_bodies,
                collide_with_areas,
                hit_info_ptr,
                max_results,
                RapierSpace2D::_is_handle_excluded_callback,
                &mut query_excluded_info,
            );
            if result_count > max_results {
                result_count = max_results;
            }

            let results_slice: &mut [godot::engine::native::PhysicsServer2DExtensionShapeResult] =
                unsafe { std::slice::from_raw_parts_mut(results, max_results) };

            for i in 0..max_results {
                let hit_info = unsafe { &mut *hit_info_ptr.add(i as usize) };

                let (rid, shape_index) =
                    RapierCollisionObject2D::get_collider_user_data(&hit_info.user_data);
                results_slice[i].rid = rid;
                results_slice[i].shape = shape_index as i32;
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

            return result_count as i32;
        }
        0
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
        let mut shape_handle = invalid_handle();
        {
            let mut lock = physics_singleton().lock().unwrap();
            let shape = lock.shapes.get_mut(&shape_rid);
            if let Some(shape) = shape {
                shape_handle = shape.get_rapier_shape();
                if !shape_handle.is_valid() {
                    return 0;
                }
            }
        }
        let max_results = max_results as usize;
        let lock = physics_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            let rapier_motion = Vector::new(motion.x, motion.y);
            let shape_info = shape_info_from_body_shape(shape_handle, transform);

            let mut query_excluded_info = default_query_excluded_info();
            query_excluded_info.query_collision_layer_mask = collision_mask;
            let mut query_exclude: Vec<Handle> = Vec::with_capacity(max_results as usize);
            query_excluded_info.query_exclude = query_exclude.as_mut_ptr();
            query_excluded_info.query_exclude_size = 0;
            let mut cpt = 0;
            let results_slice: &mut [godot::engine::native::PhysicsServer2DExtensionShapeResult] =
                unsafe { std::slice::from_raw_parts_mut(results, max_results) };
            while cpt < max_results {
                let result = shape_casting(
                    space.handle,
                    &rapier_motion,
                    shape_info.clone(),
                    collide_with_bodies,
                    collide_with_areas,
                    RapierSpace2D::_is_handle_excluded_callback,
                    &query_excluded_info,
                    false,
                );
                if !result.collided {
                    break;
                }
                query_exclude.push(result.collider);
                if !result.user_data.is_valid() {
                    continue;
                }
                let (rid, shape_index) =
                    RapierCollisionObject2D::get_collider_user_data(&result.user_data);
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

            return cpt as i32;
        }
        0
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
        let lock = physics_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            if space.locked || !space.handle.is_valid() {
                return false;
            }
            let shape = lock.shapes.get(&shape_rid);
            if let Some(shape) = shape {
                let shape_handle = shape.get_base().get_handle();
                if !shape_handle.is_valid() {
                    return false;
                }

                let rapier_motion = Vector::new(motion.x, motion.y);
                let shape_info = shape_info_from_body_shape(shape_handle, transform);

                let mut query_excluded_info = default_query_excluded_info();
                query_excluded_info.query_collision_layer_mask = collision_mask;
                let hit = shape_casting(
                    space.handle,
                    &rapier_motion,
                    shape_info,
                    collide_with_bodies,
                    collide_with_areas,
                    RapierSpace2D::_is_handle_excluded_callback,
                    &query_excluded_info,
                    false,
                )
                .toi;
                // TODO
                *closest_safe = hit as f64;
                *closest_unsafe = hit as f64;
                return true;
            }
        }
        false
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
        let lock = physics_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            if space.locked || !space.handle.is_valid() {
                return false;
            }
            let shape = lock.shapes.get(&shape_rid);
            if let Some(shape) = shape {
                let shape_handle = shape.get_base().get_handle();
                if !shape_handle.is_valid() {
                    return false;
                }

                let rapier_motion = Vector::new(motion.x, motion.y);

                let results_out = results as *mut Vector2;
                let shape_info = shape_info_from_body_shape(shape_handle, transform);
                let mut query_excluded_info = default_query_excluded_info();
                query_excluded_info.query_collision_layer_mask = collision_mask;
                let mut query_exclude: Vec<Handle> = Vec::with_capacity(max_results as usize);
                query_excluded_info.query_exclude = query_exclude.as_mut_ptr();
                query_excluded_info.query_exclude_size = 0;

                let mut array_idx = 0;
                let mut cpt = 0;

                while cpt < max_results {
                    let result = shape_casting(
                        space.handle,
                        &rapier_motion,
                        shape_info.clone(),
                        collide_with_bodies,
                        collide_with_areas,
                        RapierSpace2D::_is_handle_excluded_callback,
                        &mut query_excluded_info,
                        false,
                    );
                    if !result.collided {
                        break;
                    }
                    *result_count += 1;
                    query_exclude.push(result.collider);

                    unsafe {
                        (*results_out.add(array_idx)) =
                            Vector2::new(result.pixel_witness1.x, result.pixel_witness1.y);
                        (*results_out.add(array_idx + 1)) =
                            Vector2::new(result.pixel_witness2.x, result.pixel_witness2.y);
                    }
                    array_idx += 2;
                    cpt += 1;
                }

                return array_idx > 0;
            }
        }
        false
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
        let lock = physics_singleton().lock().unwrap();
        let space = lock.spaces.get(&self.space);
        if let Some(space) = space {
            if space.locked || !space.handle.is_valid() {
                return false;
            }
            let shape = lock.shapes.get(&shape_rid);
            if let Some(shape) = shape {
                let shape_handle = shape.get_base().get_handle();
                if !shape_handle.is_valid() {
                    return false;
                }

                let rapier_motion = Vector::new(motion.x, motion.y);
                let shape_info = shape_info_from_body_shape(shape_handle, transform);

                let mut query_excluded_info = default_query_excluded_info();
                query_excluded_info.query_collision_layer_mask = collision_mask;
                let result = shape_casting(
                    space.handle,
                    &rapier_motion,
                    shape_info,
                    collide_with_bodies,
                    collide_with_areas,
                    RapierSpace2D::_is_handle_excluded_callback,
                    &mut query_excluded_info,
                    false,
                );
                if !result.collided {
                    return false;
                }
                let (rid, shape_index) =
                    RapierCollisionObject2D::get_collider_user_data(&result.user_data);
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
                return true;
            }
        }
        false
    }
}
