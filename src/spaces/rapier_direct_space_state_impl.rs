use godot::builtin::math::ApproxEq;
use godot::classes::native::*;
use godot::prelude::*;

use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsData;
use crate::types::*;
pub struct RapierDirectSpaceStateImpl {
    pub space: Rid,
}
#[cfg(feature = "dim3")]
fn cross_product(a: Angle, b: Vector) -> Vector {
    a.cross(b)
}
#[cfg(feature = "dim2")]
fn cross_product(a: Angle, b: Vector) -> Vector {
    Vector::new(-a * b.y, a * b.x)
}
impl RapierDirectSpaceStateImpl {
    pub fn default() -> Self {
        Self {
            space: Rid::Invalid,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub unsafe fn intersect_ray(
        &mut self,
        from: Vector,
        to: Vector,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        result: *mut PhysicsServerExtensionRayResult,
        physics_data: &PhysicsData,
    ) -> bool {
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
        if !space.is_valid() {
            return false;
        }
        // if we don't do this, we end up having a zero size ray which crashes sometimes.
        // it cannot compute 0 size aabb.
        let mut to: Vector = to;
        if from.approx_eq(&to) {
            to += Vector::splat(1e-3);
        }
        // Raycast Info
        let end = to - from;
        let dir = vector_normalized(end);
        let query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let mut hit_info = RayHitInfo::default();
        let collide = physics_data.physics_engine.intersect_ray(
            space.get_handle(),
            vector_to_rapier(from),
            vector_to_rapier(dir),
            end.length(),
            collide_with_bodies,
            collide_with_areas,
            hit_from_inside,
            &mut hit_info,
            &query_excluded_info,
            &physics_data.collision_objects,
            space,
        );
        if collide {
            let result = &mut *result;
            result.position = vector_to_godot(hit_info.pixel_position);
            result.normal = vector_to_godot(hit_info.normal);
            let (rid, shape_index) =
                RapierCollisionObject::get_collider_user_data(&hit_info.user_data);
            result.rid = rid;
            result.shape = shape_index as i32;
            if let Some(collision_object_2d) = physics_data.collision_objects.get(&result.rid) {
                let instance_id = collision_object_2d.get_base().get_instance_id();
                result.collider_id = ObjectId { id: instance_id };
                if instance_id != 0 {
                    if let Ok(object) =
                        Gd::<Node>::try_from_instance_id(InstanceId::from_i64(instance_id as i64))
                    {
                        result.set_collider(object)
                    }
                }
            }
            return true;
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    pub unsafe fn intersect_point(
        &mut self,
        position: Vector,
        canvas_instance_id: u64,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut PhysicsServerExtensionShapeResult,
        max_results: i32,
        physics_data: &PhysicsData,
    ) -> i32 {
        if max_results <= 0 {
            return 0;
        }
        let max_results = max_results as usize;
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return 0;
        };
        if !space.is_valid() {
            return 0;
        }
        // Allocate memory for hit_info_array
        let mut hit_info_array = Vec::new();
        hit_info_array.resize_with(max_results, Default::default);
        let hit_info_ptr = hit_info_array.as_mut_ptr();
        // Initialize query_excluded_info
        let query_excluded_info = QueryExcludedInfo {
            query_canvas_instance_id: canvas_instance_id,
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        // Perform intersection
        let mut result_count = physics_data.physics_engine.intersect_point(
            space.get_handle(),
            vector_to_rapier(position),
            collide_with_bodies,
            collide_with_areas,
            hit_info_ptr,
            max_results,
            &query_excluded_info,
            &physics_data.collision_objects,
            space,
        );
        if result_count > max_results {
            result_count = max_results;
        }
        let results_slice: &mut [PhysicsServerExtensionShapeResult] =
            unsafe { std::slice::from_raw_parts_mut(results, max_results) };
        for (i, result_slice) in results_slice.iter_mut().enumerate().take(max_results) {
            let hit_info = unsafe { &mut *hit_info_ptr.add(i) };
            let (rid, shape_index) =
                RapierCollisionObject::get_collider_user_data(&hit_info.user_data);
            result_slice.rid = rid;
            result_slice.shape = shape_index as i32;
            let collision_object_2d = physics_data.collision_objects.get(&rid);
            if let Some(collision_object_2d) = collision_object_2d {
                let instance_id = collision_object_2d.get_base().get_instance_id();
                result_slice.collider_id = ObjectId { id: instance_id };
                if instance_id != 0 {
                    if let Ok(object) =
                        Gd::<Node>::try_from_instance_id(InstanceId::from_i64(instance_id as i64))
                    {
                        result_slice.set_collider(object)
                    }
                }
            }
        }
        result_count as i32
    }

    #[allow(clippy::too_many_arguments)]
    pub unsafe fn intersect_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut PhysicsServerExtensionShapeResult,
        max_results: i32,
        physics_data: &PhysicsData,
    ) -> i32 {
        let max_results = max_results as usize;
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return 0;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return 0;
        };
        if !space.is_valid() {
            return 0;
        }
        let shape_info = shape_info_from_body_shape(shape.get_handle(), transform);
        let mut query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let mut query_exclude = Vec::new();
        query_exclude.resize_with(max_results, Default::default);
        query_excluded_info.query_exclude = query_exclude;
        query_excluded_info.query_exclude_size = 0;
        let mut cpt = 0;
        let results_slice: &mut [PhysicsServerExtensionShapeResult] =
            unsafe { std::slice::from_raw_parts_mut(results, max_results) };
        while cpt < max_results {
            let result = physics_data.physics_engine.shape_casting(
                space.get_handle(),
                vector_to_rapier(motion),
                shape_info,
                collide_with_bodies,
                collide_with_areas,
                &query_excluded_info,
                &physics_data.collision_objects,
                space,
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
            if let Some(collision_object_2d) = physics_data.collision_objects.get(&rid) {
                results_slice[cpt].shape = shape_index as i32;
                results_slice[cpt].rid = rid;
                let instance_id = collision_object_2d.get_base().get_instance_id();
                results_slice[cpt].collider_id = ObjectId { id: instance_id };
                if instance_id != 0 {
                    if let Ok(object) =
                        Gd::<Node>::try_from_instance_id(InstanceId::from_i64(instance_id as i64))
                    {
                        results_slice[cpt].set_collider(object)
                    }
                }
                cpt += 1;
            }
        }
        cpt as i32
    }

    #[allow(clippy::too_many_arguments)]
    pub unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: *mut f64,
        closest_unsafe: *mut f64,
        physics_data: &PhysicsData,
    ) -> bool {
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return false;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
        if !space.is_valid() {
            return false;
        }
        let rapier_motion = vector_to_rapier(motion);
        let shape_info = shape_info_from_body_shape(shape.get_handle(), transform);
        let query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let hit = physics_data
            .physics_engine
            .shape_casting(
                space.get_handle(),
                rapier_motion,
                shape_info,
                collide_with_bodies,
                collide_with_areas,
                &query_excluded_info,
                &physics_data.collision_objects,
                space,
            )
            .toi;
        // TODO compute actual safe and unsafe
        let closest_safe = closest_safe as *mut real;
        *closest_safe = hit;
        let closest_unsafe = closest_unsafe as *mut real;
        *closest_unsafe = hit;
        true
    }

    #[allow(clippy::too_many_arguments)]
    pub unsafe fn collide_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut std::ffi::c_void,
        max_results: i32,
        result_count: *mut i32,
        physics_data: &PhysicsData,
    ) -> bool {
        let max_results = max_results as usize;
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return false;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
        if !space.is_valid() {
            return false;
        }
        let results_out = results as *mut Vector;
        let shape_info = shape_info_from_body_shape(shape.get_handle(), transform);
        let mut query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let mut query_exclude = Vec::new();
        query_exclude.resize_with(max_results, Default::default);
        query_excluded_info.query_exclude = query_exclude;
        query_excluded_info.query_exclude_size = 0;
        let mut array_idx = 0;
        let mut cpt = 0;
        while cpt < max_results {
            let result = physics_data.physics_engine.shape_casting(
                space.get_handle(),
                vector_to_rapier(motion),
                shape_info,
                collide_with_bodies,
                collide_with_areas,
                &query_excluded_info,
                &physics_data.collision_objects,
                space,
            );
            if !result.collided {
                break;
            }
            *result_count += 1;
            query_excluded_info.query_exclude[query_excluded_info.query_exclude_size] =
                result.collider;
            query_excluded_info.query_exclude_size += 1;
            unsafe {
                (*results_out.add(array_idx)) = vector_to_godot(result.pixel_witness2);
                (*results_out.add(array_idx + 1)) = vector_to_godot(result.pixel_witness1);
            }
            array_idx += 2;
            cpt += 1;
        }
        array_idx > 0
    }

    #[allow(clippy::too_many_arguments)]
    pub unsafe fn rest_info(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        _margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: *mut PhysicsServerExtensionShapeRestInfo,
        physics_data: &PhysicsData,
    ) -> bool {
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return false;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
        if !space.is_valid() {
            return false;
        }
        let rapier_motion = vector_to_rapier(motion);
        let shape_info = shape_info_from_body_shape(shape.get_handle(), transform);
        let query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let result = physics_data.physics_engine.shape_casting(
            space.get_handle(),
            rapier_motion,
            shape_info,
            collide_with_bodies,
            collide_with_areas,
            &query_excluded_info,
            &physics_data.collision_objects,
            space,
        );
        if !result.collided {
            return false;
        }
        let (rid, shape_index) = RapierCollisionObject::get_collider_user_data(&result.user_data);
        let r_info = &mut *rest_info;
        if let Some(collision_object_2d) = physics_data.collision_objects.get(&rid) {
            let instance_id = collision_object_2d.get_base().get_instance_id();
            r_info.collider_id = ObjectId { id: instance_id };
            if let Some(body) = collision_object_2d.get_body() {
                let rel_vec = r_info.point
                    - (body.get_base().get_transform().origin + body.get_center_of_mass());
                r_info.linear_velocity = body.get_linear_velocity(&physics_data.physics_engine)
                    + cross_product(
                        body.get_angular_velocity(&physics_data.physics_engine),
                        rel_vec,
                    );
            } else {
                r_info.linear_velocity = Vector::default();
            }
            r_info.normal = vector_to_godot(result.normal1);
            r_info.rid = rid;
            r_info.shape = shape_index as i32;
            r_info.point = vector_to_godot(result.pixel_witness1);
        }
        true
    }
}
