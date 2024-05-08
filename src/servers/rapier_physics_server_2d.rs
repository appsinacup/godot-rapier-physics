use crate::bodies::rapier_area_2d::RapierArea2D;
use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::fluids::fluid_effect_2d::FluidEffect2D;
use crate::fluids::rapier_fluid_2d::RapierFluid2D;
use crate::joints::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
use crate::joints::rapier_groove_joint_2d::RapierGrooveJoint2D;
use crate::joints::rapier_pin_joint_2d::RapierPinJoint2D;
use crate::rapier2d::query::shape_collide;
use crate::rapier2d::shape::shape_info_from_body_shape;
use crate::rapier2d::vector::Vector;
use crate::shapes::rapier_capsule_shape_2d::RapierCapsuleShape2D;
use crate::shapes::rapier_circle_shape_2d::RapierCircleShape2D;
use crate::shapes::rapier_concave_polygon_shape_2d::RapierConcavePolygonShape2D;
use crate::shapes::rapier_convex_polygon_shape_2d::RapierConvexPolygonShape2D;
use crate::shapes::rapier_rectangle_shape_2d::RapierRectangleShape2D;
use crate::shapes::rapier_segment_shape_2d::RapierSegmentShape2D;
use crate::shapes::rapier_separation_ray_shape_2d::RapierSeparationRayShape2D;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::shapes::rapier_world_boundary_shape_2d::RapierWorldBoundaryShape2D;
use crate::spaces::rapier_space_2d::RapierSpace2D;
use godot::engine::native::PhysicsServer2DExtensionMotionResult;
use godot::engine::utilities::{rid_allocate_id, rid_from_int64};
use godot::engine::IPhysicsServer2DExtension;
use godot::{engine, prelude::*};
use std::collections::HashMap;
use std::ffi::c_void;

use super::rapier_physics_singleton_2d::{bodies_singleton, fluids_singleton, joints_singleton, shapes_singleton, spaces_singleton};

#[derive(GodotClass)]
#[class(base=PhysicsServer2DExtension, init)]
pub struct RapierPhysicsServer2D {
    active: bool,
    flushing_queries: bool,
    doing_sync: bool,
    island_count: i32,
    active_objects: i32,
    collision_pairs: i32,
}

#[godot_api]
impl IPhysicsServer2DExtension for RapierPhysicsServer2D {
    fn world_boundary_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierWorldBoundaryShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn separation_ray_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierSeparationRayShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn segment_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierSegmentShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn circle_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierCircleShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn rectangle_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierRectangleShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn capsule_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierCapsuleShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn convex_polygon_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierConvexPolygonShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn concave_polygon_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierConcavePolygonShape2D::new(rid);
        shapes_singleton()
            .lock()
            .unwrap()
            .shapes
            .insert(rid, Box::new(shape));
        rid
    }
    fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        let mut lock = shapes_singleton().lock().unwrap();
        let shape = lock.shapes.get_mut(&shape);
        if let Some(shape) = shape {
            shape.set_data(data);
        }
    }
    fn shape_set_custom_solver_bias(&mut self, _shape: Rid, _bias: f32) {}

    fn shape_get_type(&self, shape: Rid) -> engine::physics_server_2d::ShapeType {
        let lock = shapes_singleton().lock().unwrap();
        let shape = lock.shapes.get(&shape);
        if let Some(shape) = shape {
            return shape.get_type();
        }
        engine::physics_server_2d::ShapeType::CUSTOM
    }
    fn shape_get_data(&self, shape: Rid) -> Variant {
        let lock = shapes_singleton().lock().unwrap();
        let shape = lock.shapes.get(&shape);
        if let Some(shape) = shape {
            return shape.get_data();
        }
        Variant::nil()
    }
    fn shape_get_custom_solver_bias(&self, _shape: Rid) -> f32 {
        0.0
    }
    unsafe fn shape_collide(
        &mut self,
        shape_a: Rid,
        xform_a: Transform2D,
        motion_a: Vector2,
        shape_b: Rid,
        xform_b: Transform2D,
        motion_b: Vector2,
        results: *mut c_void,
        result_max: i32,
        result_count: *mut i32,
    ) -> bool {
        let lock = shapes_singleton().lock().unwrap();
        let shape_a = lock.shapes.get(&shape_a);
        let shape_b = lock.shapes.get(&shape_b);
        if shape_a.is_none() || shape_b.is_none() {
            return false;
        }
        let shape_a = shape_a.unwrap();
        let shape_b = shape_b.unwrap();
        let shape_a_handle = shape_a.get_base().get_handle();
        let shape_b_handle = shape_b.get_base().get_handle();
        if !shape_a_handle.is_valid() || !shape_b_handle.is_valid() {
            return false;
        }

        let shape_a_info = shape_info_from_body_shape(shape_a_handle, xform_a);
        let shape_b_info = shape_info_from_body_shape(shape_b_handle, xform_b);
        let rapier_a_motion = Vector::new(motion_a.x, motion_a.y);
        let rapier_b_motion = Vector::new(motion_b.x, motion_b.y);

        let results_out: *mut Vector2 = results as *mut Vector2;

        let vector2_slice: &mut [Vector2] =
            unsafe { std::slice::from_raw_parts_mut(results_out, result_max as usize) };

        let result = shape_collide(
            &rapier_a_motion,
            shape_a_info,
            &rapier_b_motion,
            shape_b_info,
        );
        if !result.collided {
            return false;
        }
        *result_count = 1;

        vector2_slice[0] = Vector2::new(result.pixel_witness1.x, result.pixel_witness1.y);
        vector2_slice[1] = Vector2::new(result.pixel_witness2.x, result.pixel_witness2.y);

        return true;
    }
    fn space_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let space = RapierSpace2D::new(rid);
        spaces_singleton()
            .lock()
            .unwrap()
            .spaces
            .insert(rid, Box::new(space));
        rid
    }
    fn space_set_active(&mut self, space_rid: Rid, active: bool) {
        let mut lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&space_rid);
        let handle = space.unwrap().get_handle();
        if active {
            lock.active_spaces.insert(handle, space_rid);
        } else {
            lock.active_spaces.remove(&handle);
        }
    }
    fn space_is_active(&self, space: Rid) -> bool {
        let lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&space);
        if let Some(space) = space {
            return lock.active_spaces.contains_key(&space.get_handle());
        }
        false
    }
    fn space_set_param(
        &mut self,
        space: Rid,
        param: engine::physics_server_2d::SpaceParameter,
        value: f32,
    ) {
        let mut lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get_mut(&space);
        if let Some(space) = space {
            space.set_param(param, value);
        }
    }
    fn space_get_param(&self, space: Rid, param: engine::physics_server_2d::SpaceParameter) -> f32 {
        let lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&space);
        if let Some(space) = space {
            return space.get_param(param);
        }
        0.0
    }
    fn space_get_direct_state(
        &mut self,
        space: Rid,
    ) -> Option<Gd<engine::PhysicsDirectSpaceState2D>> {
        let lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&space);
        if let Some(space) = space {
            return space.get_direct_state();
        }
        None
        //ERR_FAIL_COND_V_MSG((using_threads && !doing_sync) || space->is_locked(), nullptr, "Space state is inaccessible right now, wait for iteration or physics process notification.");
    }
    fn space_set_debug_contacts(&mut self, space: Rid, max_contacts: i32) {
        let mut lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get_mut(&space);
        if let Some(space) = space {
            space.set_debug_contacts(max_contacts);
        }
    }
    fn space_get_contacts(&self, space: Rid) -> PackedVector2Array {
        let lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&space);
        if let Some(space) = space {
            return space.get_debug_contacts();
        }
        PackedVector2Array::new()
    }
    fn space_get_contact_count(&self, space: Rid) -> i32 {
        let lock = spaces_singleton().lock().unwrap();
        let space = lock.spaces.get(&space);
        if let Some(space) = space {
            return space.get_debug_contact_count();
        }
        0
    }
    fn area_create(&mut self) -> Rid {
        let mut lock = bodies_singleton().lock().unwrap();
        let rid = rid_from_int64(rid_allocate_id());
        let area = RapierArea2D::new(rid);
        lock.collision_objects.insert(rid, Box::new(area));
        rid
    }
    fn area_set_space(&mut self, area: Rid, space: Rid) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.set_space(space);
        }
    }
    fn area_get_space(&self, area: Rid) -> Rid {
        let lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get(&area);
        if let Some(area) = area {
            return area.get_base().get_space();
        }
        Rid::Invalid
    }
    fn area_add_shape(&mut self, area: Rid, shape: Rid, transform: Transform2D, disabled: bool) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.add_shape(shape, transform, disabled);
        }
    }
    fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.set_shape(shape_idx, shape);
        }
    }
    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform2D) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.set_shape_transform(shape_idx, transform);
        }
    }
    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.set_shape_disabled(shape_idx, disabled);
        }
    }
    fn area_get_shape_count(&self, area: Rid) -> i32 {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            return area.get_shape_count();
        }
        -1
    }
    fn area_get_shape(&self, area: Rid, shape_idx: i32) -> Rid {
        let lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get(&area);
        if let Some(area) = area {
            return area.get_shape(shape_idx);
        }
        Rid::Invalid
    }
    fn area_get_shape_transform(&self, area: Rid, shape_idx: i32) -> Transform2D {
        let lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get(&area);
        if let Some(area) = area {
            return area.get_shape_transform(shape_idx);
        }
        Transform2D::default()
    }
    fn area_remove_shape(&mut self, area: Rid, shape_idx: i32) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.remove_shape(shape_idx);
        }
    }
    fn area_clear_shapes(&mut self, area: Rid) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            while (area.get_shape_count() > 0) {
                area.remove_shape(0);
            }
        }
    }
    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.set_instance_id(id);
        }
    }
    fn area_get_object_instance_id(&self, area: Rid) -> u64 {
        let lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get(&area);
        if let Some(area) = area {
            return area.get_instance_id();
        }
        0
    }
    fn area_attach_canvas_instance_id(&mut self, area: Rid, id: u64) {
        let mut lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get_mut(&area);
        if let Some(area) = area {
            area.set_canvas_instance_id(id);
        }
    }
    fn area_get_canvas_instance_id(&self, area: Rid) -> u64 {
        let lock = bodies_singleton().lock().unwrap();
        let area = lock.collision_objects.get(&area);
        if let Some(area) = area {
            return area.get_canvas_instance_id();
        }
        0
    }
    fn area_set_param(
        &mut self,
        area: Rid,
        param: engine::physics_server_2d::AreaParameter,
        value: Variant,
    ) {
    }
    fn area_set_transform(&mut self, area: Rid, transform: Transform2D) {}
    fn area_get_param(
        &self,
        area: Rid,
        param: engine::physics_server_2d::AreaParameter,
    ) -> Variant {
        Variant::nil()
    }
    fn area_get_transform(&self, area: Rid) -> Transform2D {
        Transform2D::default()
    }
    fn area_set_collision_layer(&mut self, area: Rid, layer: u32) {}
    fn area_get_collision_layer(&self, area: Rid) -> u32 {
        0
    }
    fn area_set_collision_mask(&mut self, area: Rid, mask: u32) {}
    fn area_get_collision_mask(&self, area: Rid) -> u32 {
        0
    }
    fn area_set_monitorable(&mut self, area: Rid, monitorable: bool) {}
    fn area_set_pickable(&mut self, area: Rid, pickable: bool) {}
    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {}
    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {}
    fn body_create(&mut self) -> Rid {
        Rid::Invalid
    }
    fn body_set_space(&mut self, body: Rid, space: Rid) {}
    fn body_get_space(&self, body: Rid) -> Rid {
        Rid::Invalid
    }
    fn body_set_mode(&mut self, body: Rid, mode: engine::physics_server_2d::BodyMode) {}
    fn body_get_mode(&self, body: Rid) -> engine::physics_server_2d::BodyMode {
        engine::physics_server_2d::BodyMode::KINEMATIC
    }
    fn body_add_shape(&mut self, body: Rid, shape: Rid, transform: Transform2D, disabled: bool) {}
    fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid) {}
    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform2D) {}
    fn body_get_shape_count(&self, body: Rid) -> i32 {
        0
    }
    fn body_get_shape(&self, body: Rid, shape_idx: i32) -> Rid {
        Rid::Invalid
    }
    fn body_get_shape_transform(&self, body: Rid, shape_idx: i32) -> Transform2D {
        Transform2D::default()
    }
    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {}
    fn body_set_shape_as_one_way_collision(
        &mut self,
        body: Rid,
        shape_idx: i32,
        enable: bool,
        margin: f32,
    ) {
    }
    fn body_remove_shape(&mut self, body: Rid, shape_idx: i32) {}
    fn body_clear_shapes(&mut self, body: Rid) {}
    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {}
    fn body_get_object_instance_id(&self, body: Rid) -> u64 {
        0
    }
    fn body_attach_canvas_instance_id(&mut self, body: Rid, id: u64) {}
    fn body_get_canvas_instance_id(&self, body: Rid) -> u64 {
        0
    }
    fn body_set_continuous_collision_detection_mode(
        &mut self,
        body: Rid,
        mode: engine::physics_server_2d::CcdMode,
    ) {
    }
    fn body_get_continuous_collision_detection_mode(
        &self,
        body: Rid,
    ) -> engine::physics_server_2d::CcdMode {
        engine::physics_server_2d::CcdMode::DISABLED
    }
    fn body_set_collision_layer(&mut self, body: Rid, layer: u32) {}
    fn body_get_collision_layer(&self, body: Rid) -> u32 {
        0
    }
    fn body_set_collision_mask(&mut self, body: Rid, mask: u32) {}
    fn body_get_collision_mask(&self, body: Rid) -> u32 {
        0
    }
    fn body_set_collision_priority(&mut self, body: Rid, priority: f32) {}
    fn body_get_collision_priority(&self, body: Rid) -> f32 {
        0.0
    }
    fn body_set_param(
        &mut self,
        body: Rid,
        param: engine::physics_server_2d::BodyParameter,
        value: Variant,
    ) {
    }
    fn body_get_param(
        &self,
        body: Rid,
        param: engine::physics_server_2d::BodyParameter,
    ) -> Variant {
        Variant::nil()
    }
    fn body_reset_mass_properties(&mut self, body: Rid) {}
    fn body_set_state(
        &mut self,
        body: Rid,
        state: engine::physics_server_2d::BodyState,
        value: Variant,
    ) {
    }
    fn body_get_state(&self, body: Rid, state: engine::physics_server_2d::BodyState) -> Variant {
        Variant::nil()
    }
    fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector2) {}
    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: f32) {}
    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector2, position: Vector2) {}
    fn body_apply_central_force(&mut self, body: Rid, force: Vector2) {}
    fn body_apply_force(&mut self, body: Rid, force: Vector2, position: Vector2) {}
    fn body_apply_torque(&mut self, body: Rid, torque: f32) {}
    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector2) {}
    fn body_add_constant_force(&mut self, body: Rid, force: Vector2, position: Vector2) {}
    fn body_add_constant_torque(&mut self, body: Rid, torque: f32) {}
    fn body_set_constant_force(&mut self, body: Rid, force: Vector2) {}
    fn body_get_constant_force(&self, body: Rid) -> Vector2 {
        Vector2::default()
    }
    fn body_set_constant_torque(&mut self, body: Rid, torque: f32) {}
    fn body_get_constant_torque(&self, body: Rid) -> f32 {
        0.0
    }
    fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector2) {}
    fn body_add_collision_exception(&mut self, body: Rid, excepted_body: Rid) {}
    fn body_remove_collision_exception(&mut self, body: Rid, excepted_body: Rid) {}
    fn body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        Array::new()
    }
    fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32) {}
    fn body_get_max_contacts_reported(&self, body: Rid) -> i32 {
        0
    }
    fn body_set_contacts_reported_depth_threshold(&mut self, body: Rid, threshold: f32) {}
    fn body_get_contacts_reported_depth_threshold(&self, body: Rid) -> f32 {
        0.0
    }
    fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool) {}
    fn body_is_omitting_force_integration(&self, body: Rid) -> bool {
        false
    }
    fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable) {}
    fn body_set_force_integration_callback(
        &mut self,
        body: Rid,
        callable: Callable,
        userdata: Variant,
    ) {
    }
    unsafe fn body_collide_shape(
        &mut self,
        body: Rid,
        body_shape: i32,
        shape: Rid,
        shape_xform: Transform2D,
        motion: Vector2,
        results: *mut c_void,
        result_max: i32,
        result_count: *mut i32,
    ) -> bool {
        false
    }
    fn body_set_pickable(&mut self, body: Rid, pickable: bool) {}
    fn body_get_direct_state(&mut self, body: Rid) -> Option<Gd<engine::PhysicsDirectBodyState2D>> {
        None
    }
    unsafe fn body_test_motion(
        &self,
        body: Rid,
        from: Transform2D,
        motion: Vector2,
        margin: f32,
        collide_separation_ray: bool,
        recovery_as_collision: bool,
        result: *mut PhysicsServer2DExtensionMotionResult,
    ) -> bool {
        false
    }
    fn joint_create(&mut self) -> Rid {
        rid_from_int64(rid_allocate_id())
    }
    fn joint_clear(&mut self, joint: Rid) {
        // TODO
    }
    fn joint_set_param(
        &mut self,
        joint: Rid,
        param: engine::physics_server_2d::JointParam,
        value: f32,
    ) {
        let mut lock = joints_singleton().lock().unwrap();
        let mut joint = lock.joints.get_mut(&joint);
        if let Some(joint) = joint {
            match param {
                engine::physics_server_2d::JointParam::JOINT_PARAM_MAX_FORCE => {
                    joint.get_base().set_max_force(value);
                }
                _ => {}
            }
        }
    }
    fn joint_get_param(&self, joint: Rid, param: engine::physics_server_2d::JointParam) -> f32 {
        let lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get(&joint);
        if let Some(joint) = joint {
            match param {
                engine::physics_server_2d::JointParam::JOINT_PARAM_MAX_FORCE => {
                    return joint.get_base().get_max_force();
                }
                _ => {}
            }
        }
        0.0
    }
    fn joint_disable_collisions_between_bodies(&mut self, joint: Rid, disable: bool) {
        let mut lock = joints_singleton().lock().unwrap();
        let mut joint = lock.joints.get_mut(&joint);
        if let Some(joint) = joint {
            joint.get_base().disable_collisions_between_bodies(disable);
        }
    }
    fn joint_is_disabled_collisions_between_bodies(&self, joint: Rid) -> bool {
        let lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get(&joint);
        if let Some(joint) = joint {
            return joint.get_base().is_disabled_collisions_between_bodies();
        }
        false
    }
    fn joint_make_pin(&mut self, joint: Rid, anchor: Vector2, body_a: Rid, body_b: Rid) {
        let mut prev_joint = None;
        {
            let lock = joints_singleton().lock().unwrap();
            let joint = lock.joints.get(&joint);
            if let Some(joint) = joint {
                prev_joint = Some(joint.get_base().clone());
            }
        }
        let joint = RapierPinJoint2D::new(joint, anchor, body_a, body_b);
        if let Some(prev_joint) = prev_joint {
            joint.copy_settings_from(prev_joint);
        }
        let mut lock = joints_singleton().lock().unwrap();
        let rid = rid_from_int64(rid_allocate_id());
        lock.joints.insert(rid, Box::new(joint));
        rid
    }
    fn joint_make_groove(
        &mut self,
        joint: Rid,
        a_groove1: Vector2,
        a_groove2: Vector2,
        b_anchor: Vector2,
        body_a: Rid,
        body_b: Rid,
    ) {
        let mut prev_joint = None;
        {
            let lock = joints_singleton().lock().unwrap();
            let joint = lock.joints.get(&joint);
            if let Some(joint) = joint {
                prev_joint = Some(joint.get_base().clone());
            }
        }
        let joint = RapierGrooveJoint2D::new(joint, a_groove1, a_groove2, b_anchor, body_a, body_b);
        if let Some(prev_joint) = prev_joint {
            joint.copy_settings_from(prev_joint);
        }
        let mut lock = joints_singleton().lock().unwrap();
        let rid = rid_from_int64(rid_allocate_id());
        lock.joints.insert(rid, Box::new(joint));
        rid
    }
    fn joint_make_damped_spring(
        &mut self,
        joint: Rid,
        anchor_a: Vector2,
        anchor_b: Vector2,
        body_a: Rid,
        body_b: Rid,
    ) {
        let mut prev_joint = None;
        {
            let lock = joints_singleton().lock().unwrap();
            let joint = lock.joints.get(&joint);
            if let Some(joint) = joint {
                prev_joint = Some(joint.get_base().clone());
            }
        }
        let joint = RapierDampedSpringJoint2D::new(joint, anchor_a, anchor_b, body_a, body_b);
        if let Some(prev_joint) = prev_joint {
            joint.copy_settings_from(prev_joint);
        }
        let mut lock = joints_singleton().lock().unwrap();
        let rid = rid_from_int64(rid_allocate_id());
        lock.joints.insert(rid, Box::new(joint));
        rid

    }
    fn pin_joint_set_flag(
        &mut self,
        joint: Rid,
        flag: engine::physics_server_2d::PinJointFlag,
        enabled: bool,
    ) {
        let mut lock = joints_singleton().lock().unwrap();
        let mut joint = lock.joints.get_mut(&joint);
        if let Some(joint) = joint {
            let mut joint = joint.get_mut_pin();
            if let Some(joint) = joint {
                return joint.set_flag(flag, enabled)
            }
        }
    }
    fn pin_joint_get_flag(
        &self,
        joint: Rid,
        flag: engine::physics_server_2d::PinJointFlag,
    ) -> bool {
        let lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get(&joint);
        if let Some(joint) = joint {
            let joint = joint.get_pin();
            if let Some(joint) = joint {
                return joint.get_flag(flag)
            }
        }
        false
    }
    fn pin_joint_set_param(
        &mut self,
        joint: Rid,
        param: engine::physics_server_2d::PinJointParam,
        value: f32,
    ) {
        let mut lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get_mut(&joint);
        if let Some(joint) = joint {
            let joint = joint.get_mut_pin();
            if let Some(joint) = joint {
                return joint.set_param(param, value)
            }
        }
    }
    fn pin_joint_get_param(
        &self,
        joint: Rid,
        param: engine::physics_server_2d::PinJointParam,
    ) -> f32 {
        let lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get(&joint);
        if let Some(joint) = joint {
            let joint = joint.get_pin();
            if let Some(joint) = joint {
                return joint.get_param(param)
            }
        }
        0.0
    }
    fn damped_spring_joint_set_param(
        &mut self,
        joint: Rid,
        param: engine::physics_server_2d::DampedSpringParam,
        value: f32,
    ) {
        let mut lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get_mut(&joint);
        if let Some(joint) = joint {
            let joint = joint.get_mut_damped_spring();
            if let Some(joint) = joint {
                return joint.set_param(param, value)
            }
        }
    }
    fn damped_spring_joint_get_param(
        &self,
        joint: Rid,
        param: engine::physics_server_2d::DampedSpringParam,
    ) -> f32 {
        let lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get(&joint);
        if let Some(joint) = joint {
            let joint = joint.get_damped_spring();
            if let Some(joint) = joint {
                return joint.get_param(param)
            }
        }
        0.0
    }
    fn joint_get_type(&self, joint: Rid) -> engine::physics_server_2d::JointType {
        let lock = joints_singleton().lock().unwrap();
        let joint = lock.joints.get(&joint);
        if let Some(joint) = joint {
            return joint.get_type();
        }
        engine::physics_server_2d::JointType::MAX
    }
    fn free_rid(&mut self, rid: Rid) {
        let mut shape : Option<Box<dyn IRapierShape2D>> = None;
        {
            let mut lock = shapes_singleton().lock().unwrap();
            shape = lock.shapes.remove(&rid);
        }
        if let Some(shape) = shape {
            for (owner, _) in shape.get_base().get_owners() {
                let mut lock = bodies_singleton().lock().unwrap();
                let body = lock.collision_objects.get_mut(&owner);
                if let Some(body) = body {
                    body.remove_shape_rid(shape.get_base().get_rid());
                }
            }
            return;
        }
        let mut body : Option<Box<dyn IRapierCollisionObject2D>> = None;
        {
            let mut lock = bodies_singleton().lock().unwrap();
            body = lock.collision_objects.remove(&rid);
        }
        if let Some(body) = body {
            body.set_space(Rid::Invalid);
    
            while body.get_shape_count() > 0 {
                body.remove_shape_idx(0);
            }
            return;
        }
        {
            let mut lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.remove(&rid);
            if let Some(space) = space {
                for (handle, active_rid) in lock.active_spaces.iter_mut() {
                    if *active_rid == space.get_rid() {
                        lock.active_spaces.remove(handle);
                        break;
                    }
                }
                return;
            }
        }
        {
            let mut lock = joints_singleton().lock().unwrap();
            let joint = lock.joints.remove(&rid);
            if joint.is_some() {
                return;
            }
        }
        {
            let mut lock = fluids_singleton().lock().unwrap();
            let fluid = lock.fluids.remove(&rid);
            if fluid.is_some() {
                return;
            }
        }
    }
    fn set_active(&mut self, active: bool) {
        self.active = active;
    }
    fn init_ext(&mut self) {
        self.active = true;
        self.flushing_queries = false;
        self.doing_sync = false;
        self.island_count = 0;
        self.active_objects = 0;
        self.collision_pairs = 0;
    }
    fn step(&mut self, step: f32) {
        if !self.active {
            return;
        }
    
        self.island_count = 0;
        self.active_objects = 0;
        self.collision_pairs = 0;
        let active_spaces;
        {
            let lock = spaces_singleton().lock().unwrap();
            active_spaces = lock.active_spaces.clone();
        }
        for space in active_spaces.values() {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&space);
            if let Some(space) = space {
                space.step(step);
                self.island_count += space.get_island_count();
                self.active_objects += space.get_active_objects();
                self.collision_pairs += space.get_collision_pairs();
            }
        }
    }
    fn sync(&mut self) {
        self.doing_sync = true;
    }
    fn flush_queries(&mut self) {
        if !self.active {
            return;
        }

        self.flushing_queries = true;
        let active_spaces;
        {
            let lock = spaces_singleton().lock().unwrap();
            active_spaces = lock.active_spaces.clone();
        }
        for space in active_spaces.values() {
            let lock = spaces_singleton().lock().unwrap();
            let space = lock.spaces.get(&space);
            if let Some(space) = space {
                space.call_queries();
            }
        }

        self.flushing_queries = false;
    }
    fn end_sync(&mut self) {
        self.doing_sync = false;
    }
    fn finish(&mut self) {}
    fn is_flushing_queries(&self) -> bool {
        self.flushing_queries
    }
    fn get_process_info(&mut self, process_info: engine::physics_server_2d::ProcessInfo) -> i32 {
        match process_info {
            engine::physics_server_2d::ProcessInfo::ACTIVE_OBJECTS => return self.active_objects,
            engine::physics_server_2d::ProcessInfo::COLLISION_PAIRS => return self.collision_pairs,
            engine::physics_server_2d::ProcessInfo::ISLAND_COUNT => return self.island_count,
            _ => return 0,
        }
    }
}

#[godot_api]
impl RapierPhysicsServer2D {
    #[func]
    fn fluid_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let fluid = RapierFluid2D::new(rid);
        fluids_singleton()
            .lock()
            .unwrap()
            .fluids
            .insert(rid, Box::new(fluid));
        rid
    }

    #[func]
    fn fluid_set_space(&mut self, fluid_rid: Rid, space_rid: Rid) {
        let mut lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get_mut(&fluid_rid);
        if let Some(fluid) = fluid {
            fluid.set_space(space_rid);
        }
    }

    #[func]
    fn fluid_set_density(&mut self, fluid_rid: Rid, density: f64) {
        let mut lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get_mut(&fluid_rid);
        if let Some(fluid) = fluid {
            fluid.set_density(density);
        }
    }

    #[func]
    fn fluid_set_effects(&mut self, fluid_rid: Rid, params: Array<FluidEffect2D>) {
        let mut lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get_mut(&fluid_rid);
        if let Some(fluid) = fluid {
            fluid.set_effects(params);
        }
    }

    #[func]
    fn fluid_get_points(&self, fluid_rid: Rid) -> PackedVector2Array {
        let lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get(&fluid_rid);
        if let Some(fluid) = fluid {
            return PackedVector2Array::from(fluid.get_points().as_slice());
        }
        PackedVector2Array::default()
    }

    #[func]
    fn fluid_get_velocities(&self, fluid_rid: Rid) -> PackedVector2Array {
        let lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get(&fluid_rid);
        if let Some(fluid) = fluid {
            return PackedVector2Array::from(fluid.get_velocities().as_slice());
        }
        PackedVector2Array::default()
    }

    #[func]
    fn fluid_get_accelerations(&self, fluid_rid: Rid) -> PackedVector2Array {
        let lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get(&fluid_rid);
        if let Some(fluid) = fluid {
            return PackedVector2Array::from(fluid.get_accelerations().as_slice());
        }
        PackedVector2Array::default()
    }

    #[func]
    fn fluid_set_points(&mut self, fluid_rid: Rid, points: PackedVector2Array) {
        let mut lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get_mut(&fluid_rid);
        if let Some(fluid) = fluid {
            fluid.set_points(points.to_vec());
        }
    }

    #[func]
    fn fluid_set_points_and_velocities(&mut self, fluid_rid: Rid, points: PackedVector2Array, velocities: PackedVector2Array) {
        let mut lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get_mut(&fluid_rid);
        if let Some(fluid) = fluid {
            fluid.set_points_and_velocities(points.to_vec(), velocities.to_vec());
        }
    }

    #[func]
    fn fluid_add_points_and_velocities(&mut self, fluid_rid: Rid, points: PackedVector2Array, velocities: PackedVector2Array) {
        let mut lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get_mut(&fluid_rid);
        if let Some(fluid) = fluid {
            fluid.add_points_and_velocities(points.to_vec(), velocities.to_vec());
        }
    }

    #[func]
    fn fluid_delete_points(&mut self, fluid_rid: Rid, indices: PackedInt32Array) {
        let mut lock = fluids_singleton().lock().unwrap();
        let fluid = lock.fluids.get_mut(&fluid_rid);
        if let Some(fluid) = fluid {
            fluid.delete_points(indices.to_vec());
        }
    }
}
