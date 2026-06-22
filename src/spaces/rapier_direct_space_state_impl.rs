use godot::builtin::math::ApproxEq;
use godot::classes::native::*;
use godot::meta::conv::RawPtr;
use godot::prelude::*;
#[cfg(feature = "dim3")]
use rapier::prelude::ColliderHandle;
#[cfg(feature = "dim3")]
use rapier::prelude::FeatureId;
#[cfg(feature = "dim3")]
use rapier::prelude::Real;

use crate::bodies::rapier_collision_object::*;
use crate::bodies::rapier_collision_object_base::RapierCollisionObjectBase;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsData;
use crate::shapes::rapier_shape::IRapierShape;
use crate::types::*;
pub struct RapierDirectSpaceStateImpl {
    pub space: Rid,
}
impl Default for RapierDirectSpaceStateImpl {
    fn default() -> Self {
        Self {
            space: Rid::Invalid,
        }
    }
}
#[cfg(feature = "dim3")]
fn cross_product(a: Angle, b: Vector) -> Vector {
    a.cross(b)
}
#[cfg(feature = "dim2")]
fn cross_product(a: Angle, b: Vector) -> Vector {
    Vector::new(-a * b.y, a * b.x)
}

fn try_node_from_instance_id(instance_id: u64) -> Option<Gd<Node>> {
    let instance_id = InstanceId::try_from_i64(instance_id as i64)?;
    Gd::<Node>::try_from_instance_id(instance_id).ok()
}

impl RapierDirectSpaceStateImpl {
    #[cfg(feature = "dim3")]
    pub fn get_closest_point_to_object_volume(
        &self,
        object: Rid,
        point: Vector,
        physics_data: &PhysicsData,
    ) -> Vector {
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return Vector::ZERO;
        };
        let Some(collision_object) = physics_data.collision_objects.get(&object) else {
            return Vector::ZERO;
        };
        let base = collision_object.get_base();
        if base.get_space_id() != space.get_state().get_id() {
            return Vector::ZERO;
        }
        let Some(physics_world) = physics_data
            .physics_engine
            .get_world(space.get_state().get_id())
        else {
            return base.get_transform().origin;
        };
        let point = vector_to_rapier(point);
        let mut closest_point = point;
        let mut closest_distance = Real::MAX;
        let mut shapes_found = false;
        for shape in base.state.shapes.iter() {
            if shape.disabled || shape.collider_handle == ColliderHandle::invalid() {
                continue;
            }
            let Some(collider) = physics_world
                .physics_objects
                .collider_set
                .get(shape.collider_handle)
            else {
                continue;
            };
            shapes_found = true;
            let projection = collider
                .shape()
                .project_point(collider.position(), point, true);
            let distance = (projection.point - point).length_squared();
            if distance < closest_distance {
                closest_distance = distance;
                closest_point = projection.point;
                if closest_distance == 0.0 {
                    break;
                }
            }
        }
        if shapes_found {
            vector_to_godot(closest_point)
        } else {
            base.get_transform().origin
        }
    }

    #[allow(clippy::too_many_arguments)]
    /// # Safety
    ///
    /// `result` must point to a valid writable Godot ray result for the duration of this call.
    pub unsafe fn intersect_ray(
        &mut self,
        from: Vector,
        to: Vector,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        result: RawPtr<*mut PhysicsServerExtensionRayResult>,
        physics_data: &PhysicsData,
    ) -> bool {
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
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
            space.get_state().get_id(),
            vector_to_rapier(from),
            vector_to_rapier(dir),
            vector_length(end),
            collide_with_bodies,
            collide_with_areas,
            hit_from_inside,
            &mut hit_info,
            &query_excluded_info,
            &physics_data.collision_objects,
            &physics_data.ids,
            space,
        );
        if collide {
            let result = unsafe { &mut *result.ptr() };
            result.position = vector_to_godot(hit_info.pixel_position);
            result.normal = vector_to_godot(hit_info.normal);
            let (rid, shape_index) = RapierCollisionObjectBase::get_collider_user_data(
                &hit_info.user_data,
                &physics_data.ids,
            );
            result.rid = rid;
            result.shape = shape_index as i32;
            if let Some(collision_object_2d) = physics_data.collision_objects.get(&result.rid) {
                let instance_id = collision_object_2d.get_base().get_instance_id();
                result.collider_id = ObjectId { id: instance_id };
                if let Some(object) = try_node_from_instance_id(instance_id) {
                    unsafe { result.set_collider(object) }
                }
            }
            #[cfg(feature = "dim3")]
            match hit_info.feature {
                FeatureId::Face(i) => result.face_index = i as i32,
                FeatureId::Edge(i) => result.face_index = i as i32,
                FeatureId::Vertex(i) => result.face_index = i as i32,
                FeatureId::Unknown => result.face_index = -1,
            }
            return true;
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    /// # Safety
    ///
    /// `results` must point to writable storage for at least `max_results` Godot shape results.
    pub unsafe fn intersect_point(
        &mut self,
        position: Vector,
        canvas_instance_id: u64,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: RawPtr<*mut PhysicsServerExtensionShapeResult>,
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
        // Allocate memory for hit_info_array
        let mut hit_info_array = Vec::new();
        hit_info_array.resize_with(max_results, Default::default);
        let hit_info_ptr = hit_info_array.as_mut_ptr();
        // Initialize query_excluded_info
        let query_excluded_info = QueryExcludedInfo {
            query_canvas_instance_id: Some(canvas_instance_id),
            query_collision_layer_mask: collision_mask,
            // Godot uses point queries for 2D object picking. GDExtension does not
            // expose the pick_point flag, so point queries are the narrowest place
            // we can honor CollisionObject2D.input_pickable.
            query_pickable: true,
            ..Default::default()
        };
        // Perform intersection
        let mut result_count = physics_data.physics_engine.intersect_point(
            space.get_state().get_id(),
            vector_to_rapier(position),
            collide_with_bodies,
            collide_with_areas,
            hit_info_ptr,
            max_results,
            &query_excluded_info,
            &physics_data.collision_objects,
            &physics_data.ids,
            space,
        );
        if result_count > max_results {
            result_count = max_results;
        }
        let results_slice: &mut [PhysicsServerExtensionShapeResult] =
            unsafe { std::slice::from_raw_parts_mut(results.ptr(), max_results) };
        let mut output_count = 0;
        for i in 0..result_count {
            let hit_info = unsafe { &mut *hit_info_ptr.add(i) };
            if !hit_info.user_data.is_valid() {
                continue;
            }
            let (rid, shape_index) = RapierCollisionObjectBase::get_collider_user_data(
                &hit_info.user_data,
                &physics_data.ids,
            );
            let Some(collision_object_2d) = physics_data.collision_objects.get(&rid) else {
                continue;
            };
            let result_slice = &mut results_slice[output_count];
            result_slice.rid = rid;
            result_slice.shape = shape_index as i32;
            let instance_id = collision_object_2d.get_base().get_instance_id();
            result_slice.collider_id = ObjectId { id: instance_id };
            if let Some(object) = try_node_from_instance_id(instance_id) {
                unsafe { result_slice.set_collider(object) }
            }
            output_count += 1;
        }
        output_count as i32
    }

    #[allow(clippy::too_many_arguments)]
    /// # Safety
    ///
    /// `results` must point to writable storage for at least `max_results` Godot shape results.
    pub unsafe fn intersect_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: RawPtr<*mut PhysicsServerExtensionShapeResult>,
        max_results: i32,
        physics_data: &PhysicsData,
    ) -> i32 {
        if max_results <= 0 {
            return 0;
        }
        let max_results = max_results as usize;
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return 0;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return 0;
        };
        let shape_info = shape_info_from_body_shape(shape.get_base().get_id(), transform);
        let mut query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let mut query_exclude = Vec::new();
        query_exclude.resize_with(max_results, Default::default);
        query_excluded_info.query_exclude = query_exclude;
        query_excluded_info.query_exclude_size = 0;
        let results_slice: &mut [PhysicsServerExtensionShapeResult] =
            unsafe { std::slice::from_raw_parts_mut(results.ptr(), max_results) };
        let results: Vec<ShapeCastResult> = physics_data.physics_engine.shape_casting(
            space.get_state().get_id(),
            vector_to_rapier(motion),
            shape_info,
            margin,
            collide_with_bodies,
            collide_with_areas,
            &query_excluded_info,
            &physics_data.collision_objects,
            &physics_data.ids,
            space,
            false,
        );
        let mut cpt = 0;
        for collision in results {
            if !collision.collided {
                continue;
            }
            if !collision.user_data.is_valid() {
                continue;
            }
            let (rid, shape_index) = RapierCollisionObjectBase::get_collider_user_data(
                &collision.user_data,
                &physics_data.ids,
            );
            if let Some(collision_object_2d) = physics_data.collision_objects.get(&rid) {
                results_slice[cpt].shape = shape_index as i32;
                results_slice[cpt].rid = rid;
                let instance_id = collision_object_2d.get_base().get_instance_id();
                results_slice[cpt].collider_id = ObjectId { id: instance_id };
                if let Some(object) = try_node_from_instance_id(instance_id) {
                    unsafe { results_slice[cpt].set_collider(object) }
                }
                cpt += 1;
            }
            if cpt >= max_results {
                break;
            }
        }
        cpt as i32
    }

    #[allow(clippy::too_many_arguments)]
    /// # Safety
    ///
    /// `closest_safe` and `closest_unsafe` must point to valid writable scalar values.
    pub unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: RawPtr<*mut f64>,
        closest_unsafe: RawPtr<*mut f64>,
        physics_data: &PhysicsData,
    ) -> bool {
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return false;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
        let shape_info = shape_info_from_body_shape(shape.get_base().get_id(), transform);
        let query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let results: Vec<ShapeCastResult> = physics_data.physics_engine.shape_casting(
            space.get_state().get_id(),
            vector_to_rapier(motion),
            shape_info,
            margin,
            collide_with_bodies,
            collide_with_areas,
            &query_excluded_info,
            &physics_data.collision_objects,
            &physics_data.ids,
            space,
            true,
        );
        let mut closest_located_safe = 1.0;
        let mut closest_located_unsafe = 1.0;
        for result in results {
            if result.toi < closest_located_safe {
                closest_located_safe = result.toi;
                closest_located_unsafe = result.toi_unsafe;
            }
        }
        // If the shape is already penetrating at the start position (toi = 0),
        // Godot expects [1, 1] to indicate no blocking (shape is already inside)
        if closest_located_safe == 0.0 && closest_located_unsafe == 0.0 {
            closest_located_safe = 1.0;
            closest_located_unsafe = 1.0;
        }
        let closest_safe = closest_safe.ptr() as *mut real;
        let closest_unsafe = closest_unsafe.ptr() as *mut real;
        unsafe {
            *closest_safe = closest_located_safe;
            *closest_unsafe = closest_located_unsafe;
        }
        true
    }

    #[allow(clippy::too_many_arguments)]
    /// # Safety
    ///
    /// `results` must point to writable `Vector` pair storage for at least `max_results`
    /// contacts, and `result_count` must be null or point to a writable `i32`.
    pub unsafe fn collide_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: RawPtr<*mut std::ffi::c_void>,
        max_results: i32,
        result_count: RawPtr<*mut i32>,
        physics_data: &PhysicsData,
    ) -> bool {
        if !result_count.ptr().is_null() {
            unsafe { *result_count.ptr() = 0 };
        }
        if max_results <= 0 {
            return false;
        }
        let max_results = max_results as usize;
        let results_out = results.ptr() as *mut Vector;
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return false;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
        let shape_info = shape_info_from_body_shape(shape.get_base().get_id(), transform);
        let mut query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let mut query_exclude = Vec::new();
        query_exclude.resize_with(max_results, Default::default);
        query_excluded_info.query_exclude = query_exclude;
        query_excluded_info.query_exclude_size = 0;
        let mut intersecting_points: Vec<WitnessPair> = Vec::with_capacity(max_results);
        let results_count = physics_data.physics_engine.shape_find_intersections(
            space.get_state().get_id(),
            vector_to_rapier(motion),
            shape_info,
            margin,
            collide_with_bodies,
            collide_with_areas,
            &query_excluded_info,
            &physics_data.collision_objects,
            &physics_data.ids,
            space,
            &mut intersecting_points,
            max_results,
        );
        if !result_count.ptr().is_null() {
            unsafe { *result_count.ptr() = results_count as i32 };
        }
        if results_count == 0 {
            return false;
        }
        for (i, point) in intersecting_points.iter().enumerate().take(results_count) {
            unsafe {
                (*results_out.add(i * 2)) = vector_to_godot(point.pixel_witness1);
                (*results_out.add((i * 2) + 1)) = vector_to_godot(point.pixel_witness2);
            }
        }
        true
    }

    #[allow(clippy::too_many_arguments)]
    /// # Safety
    ///
    /// `rest_info` must point to a valid writable Godot shape rest info result.
    pub unsafe fn rest_info(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: RawPtr<*mut PhysicsServerExtensionShapeRestInfo>,
        physics_data: &PhysicsData,
    ) -> bool {
        let Some(shape) = physics_data.shapes.get(&shape_rid) else {
            return false;
        };
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return false;
        };
        let rapier_motion = vector_to_rapier(motion);
        let shape_info = shape_info_from_body_shape(shape.get_base().get_id(), transform);
        let query_excluded_info = QueryExcludedInfo {
            query_collision_layer_mask: collision_mask,
            ..Default::default()
        };
        let results = physics_data.physics_engine.shape_casting(
            space.get_state().get_id(),
            rapier_motion,
            shape_info,
            margin,
            collide_with_bodies,
            collide_with_areas,
            &query_excluded_info,
            &physics_data.collision_objects,
            &physics_data.ids,
            space,
            false,
        );
        if results.is_empty() {
            return false;
        }
        let mut found_collision: bool = false;
        let shape_position = shape_info.transform.translation;
        let mut deepest_collision_index: Option<usize> = None;
        let mut deepest_collision_distance: Option<f32> = None;
        for (i, result) in results.iter().enumerate() {
            if !result.collided {
                continue;
            }
            found_collision = true;
            let collision_distance: f32 = (result.pixel_witness2 - shape_position).length();
            if let Some(current) = deepest_collision_distance {
                if collision_distance < current {
                    deepest_collision_distance = Some(collision_distance);
                    deepest_collision_index = Some(i);
                }
            } else {
                deepest_collision_distance = Some(collision_distance);
                deepest_collision_index = Some(i);
            }
        }
        if !found_collision {
            return false;
        }
        if let Some(i) = deepest_collision_index {
            let deepest_collision: &ShapeCastResult = &results[i];
            if !deepest_collision.user_data.is_valid() {
                return false;
            }
            let (rid, shape_index) = RapierCollisionObjectBase::get_collider_user_data(
                &deepest_collision.user_data,
                &physics_data.ids,
            );
            let r_info = unsafe { &mut *rest_info.ptr() };
            let Some(collision_object_2d) = physics_data.collision_objects.get(&rid) else {
                return false;
            };
            let instance_id = collision_object_2d.get_base().get_instance_id();
            r_info.collider_id = ObjectId { id: instance_id };
            let collision_point = vector_to_godot(deepest_collision.pixel_witness2);
            if let Some(body) = collision_object_2d.get_body() {
                let rel_vec = collision_point
                    - (body.get_base().get_transform().origin + body.get_center_of_mass());
                r_info.linear_velocity = body.get_linear_velocity(&physics_data.physics_engine)
                    + cross_product(
                        body.get_angular_velocity(&physics_data.physics_engine),
                        rel_vec,
                    );
            } else {
                r_info.linear_velocity = Vector::default();
            }
            r_info.normal = vector_to_godot(deepest_collision.normal2);
            r_info.rid = rid;
            r_info.shape = shape_index as i32;
            r_info.point = collision_point;
            return true;
        }
        false
    }
}
