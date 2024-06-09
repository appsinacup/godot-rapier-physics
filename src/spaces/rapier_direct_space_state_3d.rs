use std::ops::Deref;

use crate::{
    bodies::rapier_collision_object::{IRapierCollisionObject, RapierCollisionObject},
    rapier_wrapper::{
        convert::{vector_to_godot, vector_to_rapier},
        handle::Handle,
        query::{intersect_point, shape_casting, PointHitInfo, QueryExcludedInfo, RayHitInfo},
        shape::shape_info_from_body_shape,
        user_data::UserData,
    },
    servers::rapier_physics_singleton::{
        active_spaces_singleton, bodies_singleton, shapes_singleton, spaces_singleton,
    },
    spaces::rapier_space::RapierSpace,
    Vector,
};
use godot::{
    engine::{
        native::{ObjectId, PhysicsServer3DExtensionShapeRestInfo},
        IPhysicsDirectSpaceState3DExtension, PhysicsDirectSpaceState3DExtension,
    },
    prelude::*,
};
use rapier::math::Real;

use super::rapier_space_body_helper::is_handle_excluded_callback;
pub type RapierDirectSpaceState = RapierDirectSpaceState3D;

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState3DExtension,tool)]
pub struct RapierDirectSpaceState3D {
    space: Rid,
    base: Base<PhysicsDirectSpaceState3DExtension>,
}

impl RapierDirectSpaceState3D {
    pub fn set_space(&mut self, space: Rid) {
        self.space = space;
    }
}

#[godot_api]
impl IPhysicsDirectSpaceState3DExtension for RapierDirectSpaceState3D {
    fn init(base: Base<PhysicsDirectSpaceState3DExtension>) -> Self {
        Self {
            space: Rid::Invalid,
            base,
        }
    }

    unsafe fn intersect_ray(
        &mut self,
        from: Vector,
        to: Vector,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        _hit_back_faces: bool,
        _pick_ray: bool,
        result: *mut godot::engine::native::PhysicsServer3DExtensionRayResult,
    ) -> bool {
        if let Some(space) = spaces_singleton().spaces.get(&self.space) {
            if !space.is_valid() {
                return false;
            }
            // Raycast Info
            let end = to - from;
            let dir = end.normalized();

            let rapier_from = vector_to_rapier(from);
            let rapier_dir = vector_to_rapier(dir);

            let mut query_excluded_info = QueryExcludedInfo::default();
            query_excluded_info.query_collision_layer_mask = collision_mask;

            let mut hit_info = RayHitInfo::default();
            let collide = crate::rapier_wrapper::query::intersect_ray(
                space.get_handle(),
                rapier_from,
                rapier_dir,
                end.length(),
                collide_with_bodies,
                collide_with_areas,
                hit_from_inside,
                &mut hit_info,
                is_handle_excluded_callback,
                &query_excluded_info,
            );

            if collide {
                let result = &mut *result;
                result.position = vector_to_godot(hit_info.pixel_position);
                result.normal = vector_to_godot(hit_info.normal);
                let (rid, shape_index) =
                    RapierCollisionObject::get_collider_user_data(&hit_info.user_data);
                result.rid = rid;
                result.shape = shape_index as i32;
                if let Some(collision_object_2d) =
                    bodies_singleton().collision_objects.get(&result.rid)
                {
                    let instance_id = collision_object_2d.get_base().get_instance_id();
                    result.collider_id = ObjectId { id: instance_id };
                    if instance_id != 0 {
                        result.collider = RapierSpace::_get_object_instance_hack(instance_id);
                    }
                }

                return true;
            }
        }
        false
    }

    unsafe fn intersect_point(
        &mut self,
        position: Vector,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::engine::native::PhysicsServer3DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        let max_results = max_results as usize;
        if max_results <= 0 {
            return 0;
        }
        if let Some(space) = spaces_singleton().spaces.get(&self.space) {
            if space.is_valid() {
                return 0;
            }
            let rapier_pos = vector_to_rapier(position);

            // Allocate memory for hit_info_array
            let mut hit_info_array: Vec<PointHitInfo> = Vec::with_capacity(max_results);
            let hit_info_ptr = hit_info_array.as_mut_ptr();

            // Initialize query_excluded_info
            let mut query_excluded_info = QueryExcludedInfo::default();
            query_excluded_info.query_collision_layer_mask = collision_mask;

            // Perform intersection
            let mut result_count = intersect_point(
                space.get_handle(),
                rapier_pos,
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

            let results_slice: &mut [godot::engine::native::PhysicsServer3DExtensionShapeResult] =
                unsafe { std::slice::from_raw_parts_mut(results, max_results) };

            for i in 0..max_results {
                let hit_info = unsafe { &mut *hit_info_ptr.add(i) };

                let (rid, shape_index) =
                    RapierCollisionObject::get_collider_user_data(&hit_info.user_data);
                results_slice[i].rid = rid;
                results_slice[i].shape = shape_index as i32;
                let lock = bodies_singleton();
                let collision_object_2d = lock.collision_objects.get(&rid);
                if let Some(collision_object_2d) = collision_object_2d {
                    let instance_id = collision_object_2d.get_base().get_instance_id();
                    results_slice[i].collider_id = ObjectId { id: instance_id };

                    if instance_id != 0 {
                        results_slice[i].collider =
                            RapierSpace::_get_object_instance_hack(instance_id);
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
        transform: Transform3D,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::engine::native::PhysicsServer3DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        let max_results = max_results as usize;
        if let Some(shape) = shapes_singleton().shapes.get_mut(&shape_rid) {
            if !shape.get_base().is_valid() {
                return 0;
            }
            if let Some(space) = spaces_singleton().spaces.get(&self.space) {
                if !space.is_valid() {
                    return 0;
                }
                let rapier_motion = vector_to_rapier(motion);
                let shape_info =
                    shape_info_from_body_shape(shape.get_base().get_handle(), transform);

                let mut query_excluded_info = QueryExcludedInfo::default();
                query_excluded_info.query_collision_layer_mask = collision_mask;
                let query_exclude: Vec<Handle> = Vec::with_capacity(max_results);
                query_excluded_info.query_exclude = query_exclude;
                query_excluded_info.query_exclude_size = 0;
                let mut cpt = 0;
                let results_slice: &mut [godot::engine::native::PhysicsServer3DExtensionShapeResult] =
            unsafe { std::slice::from_raw_parts_mut(results, max_results) };
                while cpt < max_results {
                    let result = shape_casting(
                        space.get_handle(),
                        rapier_motion,
                        shape_info,
                        collide_with_bodies,
                        collide_with_areas,
                        is_handle_excluded_callback,
                        &query_excluded_info,
                    );
                    if !result.collided {
                        break;
                    }
                    query_excluded_info.query_exclude[query_excluded_info.query_exclude_size] =
                        result.collider;
                    query_excluded_info.query_exclude_size += 1;
                    if !result.user_data.is_valid() {
                        continue;
                    }
                    let (rid, shape_index) =
                        RapierCollisionObject::get_collider_user_data(&result.user_data);
                    let lock = bodies_singleton();
                    let collision_object_2d = lock.collision_objects.get(&rid);
                    if let Some(collision_object_2d) = collision_object_2d {
                        results_slice[cpt].shape = shape_index as i32;
                        results_slice[cpt].rid = rid;

                        let instance_id = collision_object_2d.get_base().get_instance_id();
                        results_slice[cpt].collider_id = ObjectId { id: instance_id };

                        if instance_id != 0 {
                            results_slice[cpt].collider =
                                RapierSpace::_get_object_instance_hack(instance_id);
                        }
                        cpt += 1;
                    }
                }

                return cpt as i32;
            }
        }
        0
    }

    unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: *mut f64,
        closest_unsafe: *mut f64,
        _info: *mut PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
        if let Some(shape) = shapes_singleton().shapes.get(&shape_rid) {
            if !shape.get_base().is_valid() {
                return false;
            }
            if let Some(space) = spaces_singleton().spaces.get(&self.space) {
                if !space.is_valid() {
                    return false;
                }
                let rapier_motion = vector_to_rapier(motion);
                let shape_info =
                    shape_info_from_body_shape(shape.get_base().get_handle(), transform);

                let mut query_excluded_info = QueryExcludedInfo::default();
                query_excluded_info.query_collision_layer_mask = collision_mask;
                let hit = shape_casting(
                    space.get_handle(),
                    rapier_motion,
                    shape_info,
                    collide_with_bodies,
                    collide_with_areas,
                    is_handle_excluded_callback,
                    &query_excluded_info,
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
        transform: Transform3D,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut std::ffi::c_void,
        max_results: i32,
        result_count: *mut i32,
    ) -> bool {
        if let Some(shape) = shapes_singleton().shapes.get(&shape_rid) {
            if !shape.get_base().is_valid() {
                return false;
            }
            if let Some(space) = spaces_singleton().spaces.get(&self.space) {
                if !space.is_valid() {
                    return false;
                }
                let rapier_motion = vector_to_rapier(motion);

                let results_out = results as *mut Vector;
                let shape_info =
                    shape_info_from_body_shape(shape.get_base().get_handle(), transform);
                let mut query_excluded_info = QueryExcludedInfo::default();
                query_excluded_info.query_collision_layer_mask = collision_mask;
                let query_exclude: Vec<Handle> = Vec::with_capacity(max_results as usize);
                query_excluded_info.query_exclude = query_exclude;
                query_excluded_info.query_exclude_size = 0;

                let mut array_idx = 0;
                let mut cpt = 0;

                while cpt < max_results {
                    let result = shape_casting(
                        space.get_handle(),
                        rapier_motion,
                        shape_info,
                        collide_with_bodies,
                        collide_with_areas,
                        is_handle_excluded_callback,
                        &mut query_excluded_info,
                    );
                    if !result.collided {
                        break;
                    }
                    *result_count += 1;
                    query_excluded_info.query_exclude[query_excluded_info.query_exclude_size] =
                        result.collider;
                    query_excluded_info.query_exclude_size += 1;
                    unsafe {
                        (*results_out.add(array_idx)) = vector_to_godot(result.pixel_witness1);
                        (*results_out.add(array_idx + 1)) = vector_to_godot(result.pixel_witness2);
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
        transform: Transform3D,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: *mut godot::engine::native::PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
        if let Some(shape) = shapes_singleton().shapes.get(&shape_rid) {
            if !shape.get_base().is_valid() {
                return false;
            }
            if let Some(space) = spaces_singleton().spaces.get(&self.space) {
                if !space.is_valid() {
                    return false;
                }
                let rapier_motion = vector_to_rapier(motion);
                let shape_info =
                    shape_info_from_body_shape(shape.get_base().get_handle(), transform);

                let mut query_excluded_info = QueryExcludedInfo::default();
                query_excluded_info.query_collision_layer_mask = collision_mask;
                let result = shape_casting(
                    space.get_handle(),
                    rapier_motion,
                    shape_info,
                    collide_with_bodies,
                    collide_with_areas,
                    is_handle_excluded_callback,
                    &mut query_excluded_info,
                );
                if !result.collided {
                    return false;
                }
                let (rid, shape_index) =
                    RapierCollisionObject::get_collider_user_data(&result.user_data);
                let lock = bodies_singleton();
                let collision_object_2d = lock.collision_objects.get(&rid);
                let r_info = &mut *rest_info;
                if let Some(collision_object_2d) = collision_object_2d {
                    let instance_id = collision_object_2d.get_base().get_instance_id();
                    r_info.collider_id = ObjectId { id: instance_id };
                    if let Some(body) = collision_object_2d.get_body() {
                        let rel_vec = r_info.point
                            - (body.get_base().get_transform().origin + body.get_center_of_mass());
                        //r_info.linear_velocity = vector_to_godot(
                        //    -body.get_angular_velocity() * rel_vec.y,
                        //    body.get_angular_velocity() * rel_vec.x,
                        //) + body.get_linear_velocity();
                    } else {
                        r_info.linear_velocity = Vector::ZERO
                    }
                    r_info.normal = vector_to_godot(result.normal1);
                    r_info.rid = rid;
                    r_info.shape = shape_index as i32;
                }
                return true;
            }
        }
        false
    }
}
