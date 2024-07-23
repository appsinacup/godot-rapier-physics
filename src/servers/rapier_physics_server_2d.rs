use std::ffi::c_void;

use godot::classes::physics_server_2d::*;
use godot::classes::IPhysicsServer2DExtension;
use godot::classes::PhysicsServer2DExtension;
use godot::classes::{self};
use godot::prelude::*;

use super::rapier_physics_server_impl::RapierPhysicsServerImpl;
use super::rapier_physics_singleton::physics_data;
use crate::types::*;
#[derive(GodotClass, Default)]
#[class(base=Object,init,tool)]
pub struct RapierPhysicsServerFactory2D;
#[godot_api]
impl RapierPhysicsServerFactory2D {
    #[func]
    fn create_server() -> Gd<RapierPhysicsServer2D> {
        RapierPhysicsServer2D::new_alloc()
    }
}
#[derive(GodotClass)]
#[class(base=PhysicsServer2DExtension, tool)]
pub struct RapierPhysicsServer2D {
    pub implementation: RapierPhysicsServerImpl,
    base: Base<PhysicsServer2DExtension>,
}
#[godot_api]
impl IPhysicsServer2DExtension for RapierPhysicsServer2D {
    fn init(base: Base<PhysicsServer2DExtension>) -> Self {
        Self {
            implementation: RapierPhysicsServerImpl::default(),
            base,
        }
    }

    fn world_boundary_shape_create(&mut self) -> Rid {
        self.implementation.world_boundary_shape_create()
    }

    fn separation_ray_shape_create(&mut self) -> Rid {
        self.implementation.separation_ray_shape_create()
    }

    fn segment_shape_create(&mut self) -> Rid {
        self.implementation.segment_shape_create()
    }

    fn circle_shape_create(&mut self) -> Rid {
        self.implementation.circle_shape_create()
    }

    fn rectangle_shape_create(&mut self) -> Rid {
        self.implementation.rectangle_shape_create()
    }

    fn capsule_shape_create(&mut self) -> Rid {
        self.implementation.capsule_shape_create()
    }

    fn convex_polygon_shape_create(&mut self) -> Rid {
        self.implementation.convex_polygon_shape_create()
    }

    fn concave_polygon_shape_create(&mut self) -> Rid {
        self.implementation.concave_polygon_shape_create()
    }

    fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        self.implementation.shape_set_data(shape, data)
    }

    fn shape_set_custom_solver_bias(&mut self, _shape: Rid, _bias: f32) {}

    fn shape_get_type(&self, shape: Rid) -> ShapeType {
        self.implementation.shape_get_type(shape)
    }

    fn shape_get_data(&self, shape: Rid) -> Variant {
        self.implementation.shape_get_data(shape)
    }

    fn shape_get_custom_solver_bias(&self, shape: Rid) -> f32 {
        self.implementation.shape_get_custom_solver_bias(shape)
    }

    unsafe fn shape_collide(
        &mut self,
        shape_a: Rid,
        xform_a: Transform,
        motion_a: Vector,
        shape_b: Rid,
        xform_b: Transform,
        motion_b: Vector,
        results: *mut c_void,
        result_max: i32,
        result_count: *mut i32,
    ) -> bool {
        self.implementation.shape_collide(
            shape_a,
            xform_a,
            motion_a,
            shape_b,
            xform_b,
            motion_b,
            results,
            result_max,
            result_count,
        )
    }

    fn space_create(&mut self) -> Rid {
        self.implementation.space_create()
    }

    fn space_set_active(&mut self, space_rid: Rid, active: bool) {
        self.implementation.space_set_active(space_rid, active)
    }

    fn space_is_active(&self, space: Rid) -> bool {
        self.implementation.space_is_active(space)
    }

    fn space_set_param(&mut self, space: Rid, param: SpaceParameter, value: f32) {
        self.implementation.space_set_param(space, param, value)
    }

    fn space_get_param(&self, space: Rid, param: SpaceParameter) -> f32 {
        self.implementation.space_get_param(space, param)
    }

    fn space_get_direct_state(
        &mut self,
        space: Rid,
    ) -> Option<Gd<classes::PhysicsDirectSpaceState2D>> {
        self.implementation.space_get_direct_state(space)
    }

    fn space_set_debug_contacts(&mut self, space: Rid, max_contacts: i32) {
        self.implementation
            .space_set_debug_contacts(space, max_contacts)
    }

    fn space_get_contacts(&self, space: Rid) -> PackedVectorArray {
        self.implementation.space_get_contacts(space)
    }

    fn space_get_contact_count(&self, space: Rid) -> i32 {
        self.implementation.space_get_contact_count(space)
    }

    fn area_create(&mut self) -> Rid {
        self.implementation.area_create()
    }

    fn area_set_space(&mut self, area: Rid, space: Rid) {
        self.implementation.area_set_space(area, space)
    }

    fn area_get_space(&self, area: Rid) -> Rid {
        self.implementation.area_get_space(area)
    }

    fn area_add_shape(&mut self, area: Rid, shape: Rid, transform: Transform, disabled: bool) {
        self.implementation
            .area_add_shape(area, shape, transform, disabled)
    }

    fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid) {
        self.implementation.area_set_shape(area, shape_idx, shape)
    }

    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform) {
        self.implementation
            .area_set_shape_transform(area, shape_idx, transform)
    }

    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        self.implementation
            .area_set_shape_disabled(area, shape_idx, disabled)
    }

    fn area_get_shape_count(&self, area: Rid) -> i32 {
        self.implementation.area_get_shape_count(area)
    }

    fn area_get_shape(&self, area: Rid, shape_idx: i32) -> Rid {
        self.implementation.area_get_shape(area, shape_idx)
    }

    fn area_get_shape_transform(&self, area: Rid, shape_idx: i32) -> Transform {
        self.implementation
            .area_get_shape_transform(area, shape_idx)
    }

    fn area_remove_shape(&mut self, area: Rid, shape_idx: i32) {
        self.implementation.area_remove_shape(area, shape_idx)
    }

    fn area_clear_shapes(&mut self, area: Rid) {
        self.implementation.area_clear_shapes(area)
    }

    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        self.implementation.area_attach_object_instance_id(area, id)
    }

    fn area_get_object_instance_id(&self, area: Rid) -> u64 {
        self.implementation.area_get_object_instance_id(area)
    }

    fn area_attach_canvas_instance_id(&mut self, area: Rid, id: u64) {
        self.implementation.area_attach_canvas_instance_id(area, id)
    }

    fn area_get_canvas_instance_id(&self, area: Rid) -> u64 {
        self.implementation.area_get_canvas_instance_id(area)
    }

    fn area_set_param(&mut self, area: Rid, param: AreaParameter, value: Variant) {
        self.implementation.area_set_param(area, param, value);
    }

    fn area_set_transform(&mut self, area: Rid, transform: Transform) {
        self.implementation.area_set_transform(area, transform);
    }

    fn area_get_param(&self, area: Rid, param: AreaParameter) -> Variant {
        self.implementation.area_get_param(area, param)
    }

    fn area_get_transform(&self, area: Rid) -> Transform {
        self.implementation.area_get_transform(area)
    }

    fn area_set_collision_layer(&mut self, area: Rid, layer: u32) {
        self.implementation.area_set_collision_layer(area, layer);
    }

    fn area_get_collision_layer(&self, area: Rid) -> u32 {
        self.implementation.area_get_collision_layer(area)
    }

    fn area_set_collision_mask(&mut self, area: Rid, mask: u32) {
        self.implementation.area_set_collision_mask(area, mask);
    }

    fn area_get_collision_mask(&self, area: Rid) -> u32 {
        self.implementation.area_get_collision_mask(area)
    }

    fn area_set_monitorable(&mut self, area: Rid, monitorable: bool) {
        self.implementation.area_set_monitorable(area, monitorable);
    }

    fn area_set_pickable(&mut self, area: Rid, pickable: bool) {
        self.implementation.area_set_pickable(area, pickable);
    }

    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {
        self.implementation
            .area_set_monitor_callback(area, callback);
    }

    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {
        self.implementation
            .area_set_area_monitor_callback(area, callback);
    }

    fn body_create(&mut self) -> Rid {
        self.implementation.body_create()
    }

    fn body_set_space(&mut self, body: Rid, space: Rid) {
        self.implementation.body_set_space(body, space);
    }

    fn body_get_space(&self, body: Rid) -> Rid {
        self.implementation.body_get_space(body)
    }

    fn body_set_mode(&mut self, body: Rid, mode: BodyMode) {
        self.implementation.body_set_mode(body, mode);
    }

    fn body_get_mode(&self, body: Rid) -> BodyMode {
        self.implementation.body_get_mode(body)
    }

    fn body_add_shape(&mut self, body: Rid, shape: Rid, transform: Transform, disabled: bool) {
        self.implementation
            .body_add_shape(body, shape, transform, disabled);
    }

    fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid) {
        self.implementation.body_set_shape(body, shape_idx, shape);
    }

    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform) {
        self.implementation
            .body_set_shape_transform(body, shape_idx, transform);
    }

    fn body_get_shape_count(&self, body: Rid) -> i32 {
        self.implementation.body_get_shape_count(body)
    }

    fn body_get_shape(&self, body: Rid, shape_idx: i32) -> Rid {
        self.implementation.body_get_shape(body, shape_idx)
    }

    fn body_get_shape_transform(&self, body: Rid, shape_idx: i32) -> Transform {
        self.implementation
            .body_get_shape_transform(body, shape_idx)
    }

    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {
        self.implementation
            .body_set_shape_disabled(body, shape_idx, disabled)
    }

    fn body_set_shape_as_one_way_collision(
        &mut self,
        body: Rid,
        shape_idx: i32,
        enable: bool,
        margin: f32,
    ) {
        self.implementation
            .body_set_shape_as_one_way_collision(body, shape_idx, enable, margin);
    }

    fn body_remove_shape(&mut self, body: Rid, shape_idx: i32) {
        self.implementation.body_remove_shape(body, shape_idx);
    }

    fn body_clear_shapes(&mut self, body: Rid) {
        self.implementation.body_clear_shapes(body);
    }

    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {
        self.implementation.body_attach_object_instance_id(body, id);
    }

    fn body_get_object_instance_id(&self, body: Rid) -> u64 {
        self.implementation.body_get_object_instance_id(body)
    }

    fn body_attach_canvas_instance_id(&mut self, body: Rid, id: u64) {
        self.implementation.body_attach_canvas_instance_id(body, id);
    }

    fn body_get_canvas_instance_id(&self, body: Rid) -> u64 {
        self.implementation.body_get_canvas_instance_id(body)
    }

    fn body_set_continuous_collision_detection_mode(&mut self, body: Rid, mode: CcdMode) {
        self.implementation
            .body_set_continuous_collision_detection_mode(body, mode);
    }

    fn body_get_continuous_collision_detection_mode(&self, body: Rid) -> CcdMode {
        self.implementation
            .body_get_continuous_collision_detection_mode(body)
    }

    fn body_set_collision_layer(&mut self, body: Rid, layer: u32) {
        self.implementation.body_set_collision_layer(body, layer);
    }

    fn body_get_collision_layer(&self, body: Rid) -> u32 {
        self.implementation.body_get_collision_layer(body)
    }

    fn body_set_collision_mask(&mut self, body: Rid, mask: u32) {
        self.implementation.body_set_collision_mask(body, mask);
    }

    fn body_get_collision_mask(&self, body: Rid) -> u32 {
        self.implementation.body_get_collision_mask(body)
    }

    fn body_set_collision_priority(&mut self, body: Rid, priority: f32) {
        self.implementation
            .body_set_collision_priority(body, priority);
    }

    fn body_get_collision_priority(&self, body: Rid) -> f32 {
        self.implementation.body_get_collision_priority(body)
    }

    fn body_set_param(&mut self, body: Rid, param: BodyParameter, value: Variant) {
        self.implementation.body_set_param(body, param, value);
    }

    fn body_get_param(&self, body: Rid, param: BodyParameter) -> Variant {
        self.implementation.body_get_param(body, param)
    }

    fn body_reset_mass_properties(&mut self, body: Rid) {
        self.implementation.body_reset_mass_properties(body);
    }

    fn body_set_state(&mut self, body: Rid, state: BodyState, value: Variant) {
        self.implementation.body_set_state(body, state, value);
    }

    fn body_get_state(&self, body: Rid, state: BodyState) -> Variant {
        self.implementation.body_get_state(body, state)
    }

    fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector) {
        self.implementation
            .body_apply_central_impulse(body, impulse);
    }

    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: f32) {
        self.implementation.body_apply_torque_impulse(body, impulse);
    }

    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector, position: Vector) {
        self.implementation
            .body_apply_impulse(body, impulse, position);
    }

    fn body_apply_central_force(&mut self, body: Rid, force: Vector) {
        self.implementation.body_apply_central_force(body, force);
    }

    fn body_apply_force(&mut self, body: Rid, force: Vector, position: Vector) {
        self.implementation.body_apply_force(body, force, position);
    }

    fn body_apply_torque(&mut self, body: Rid, torque: Angle) {
        self.implementation.body_apply_torque(body, torque);
    }

    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector) {
        self.implementation
            .body_add_constant_central_force(body, force);
    }

    fn body_add_constant_force(&mut self, body: Rid, force: Vector, position: Vector) {
        self.implementation
            .body_add_constant_force(body, force, position);
    }

    fn body_add_constant_torque(&mut self, body: Rid, torque: Angle) {
        self.implementation.body_add_constant_torque(body, torque);
    }

    fn body_set_constant_force(&mut self, body: Rid, force: Vector) {
        self.implementation.body_set_constant_force(body, force);
    }

    fn body_get_constant_force(&self, body: Rid) -> Vector {
        self.implementation.body_get_constant_force(body)
    }

    fn body_set_constant_torque(&mut self, body: Rid, torque: Angle) {
        self.implementation.body_set_constant_torque(body, torque);
    }

    fn body_get_constant_torque(&self, body: Rid) -> f32 {
        self.implementation.body_get_constant_torque(body)
    }

    fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector) {
        self.implementation
            .body_set_axis_velocity(body, axis_velocity);
    }

    fn body_add_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        self.implementation
            .body_add_collision_exception(body, excepted_body);
    }

    fn body_remove_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        self.implementation
            .body_remove_collision_exception(body, excepted_body);
    }

    fn body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        self.implementation.body_get_collision_exceptions(body)
    }

    fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32) {
        self.implementation
            .body_set_max_contacts_reported(body, amount);
    }

    fn body_get_max_contacts_reported(&self, body: Rid) -> i32 {
        self.implementation.body_get_max_contacts_reported(body)
    }

    fn body_set_contacts_reported_depth_threshold(&mut self, body: Rid, threshold: f32) {
        self.implementation
            .body_set_contacts_reported_depth_threshold(body, threshold);
    }

    fn body_get_contacts_reported_depth_threshold(&self, body: Rid) -> f32 {
        self.implementation
            .body_get_contacts_reported_depth_threshold(body)
    }

    fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool) {
        self.implementation
            .body_set_omit_force_integration(body, enable);
    }

    fn body_is_omitting_force_integration(&self, body: Rid) -> bool {
        self.implementation.body_is_omitting_force_integration(body)
    }

    fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable) {
        self.implementation
            .body_set_state_sync_callback(body, callable);
    }

    fn body_set_force_integration_callback(
        &mut self,
        body: Rid,
        callable: Callable,
        userdata: Variant,
    ) {
        self.implementation
            .body_set_force_integration_callback(body, callable, userdata);
    }

    unsafe fn body_collide_shape(
        &mut self,
        body: Rid,
        body_shape: i32,
        shape: Rid,
        shape_xform: Transform,
        motion: Vector,
        results: *mut c_void,
        result_max: i32,
        result_count: *mut i32,
    ) -> bool {
        self.implementation.body_collide_shape(
            body,
            body_shape,
            shape,
            shape_xform,
            motion,
            results,
            result_max,
            result_count,
        )
    }

    fn body_set_pickable(&mut self, body: Rid, pickable: bool) {
        self.implementation.body_set_pickable(body, pickable);
    }

    fn body_get_direct_state(&mut self, body: Rid) -> Option<Gd<PhysicsDirectBodyState>> {
        self.implementation.body_get_direct_state(body)
    }

    unsafe fn body_test_motion(
        &self,
        body: Rid,
        from: Transform,
        motion: Vector,
        margin: f32,
        collide_separation_ray: bool,
        recovery_as_collision: bool,
        result: *mut PhysicsServerExtensionMotionResult,
    ) -> bool {
        self.implementation.body_test_motion(
            body,
            from,
            motion,
            margin,
            1,
            collide_separation_ray,
            recovery_as_collision,
            result,
        )
    }

    fn joint_create(&mut self) -> Rid {
        self.implementation.joint_create()
    }

    fn joint_clear(&mut self, rid: Rid) {
        self.implementation.joint_clear(rid);
    }

    fn joint_set_param(&mut self, joint: Rid, param: JointParam, value: f32) {
        self.implementation.joint_set_param(joint, param, value);
    }

    fn joint_get_param(&self, joint: Rid, param: JointParam) -> f32 {
        self.implementation.joint_get_param(joint, param)
    }

    fn joint_disable_collisions_between_bodies(&mut self, joint: Rid, disable: bool) {
        self.implementation
            .joint_disable_collisions_between_bodies(joint, disable);
    }

    fn joint_is_disabled_collisions_between_bodies(&self, joint: Rid) -> bool {
        self.implementation
            .joint_is_disabled_collisions_between_bodies(joint)
    }

    fn joint_make_pin(&mut self, rid: Rid, anchor: Vector, body_a: Rid, body_b: Rid) {
        self.implementation
            .joint_make_pin(rid, anchor, body_a, body_b);
    }

    fn joint_make_groove(
        &mut self,
        rid: Rid,
        a_groove1: Vector,
        a_groove2: Vector,
        b_anchor: Vector,
        body_a: Rid,
        body_b: Rid,
    ) {
        self.implementation
            .joint_make_groove(rid, a_groove1, a_groove2, b_anchor, body_a, body_b);
    }

    fn joint_make_damped_spring(
        &mut self,
        rid: Rid,
        anchor_a: Vector,
        anchor_b: Vector,
        body_a: Rid,
        body_b: Rid,
    ) {
        self.implementation
            .joint_make_damped_spring(rid, anchor_a, anchor_b, body_a, body_b);
    }

    fn pin_joint_set_flag(&mut self, joint: Rid, flag: PinJointFlag, enabled: bool) {
        self.implementation.pin_joint_set_flag(joint, flag, enabled);
    }

    fn pin_joint_get_flag(&self, joint: Rid, flag: PinJointFlag) -> bool {
        self.implementation.pin_joint_get_flag(joint, flag)
    }

    fn pin_joint_set_param(&mut self, joint: Rid, param: PinJointParam, value: f32) {
        self.implementation.pin_joint_set_param(joint, param, value);
    }

    fn pin_joint_get_param(&self, joint: Rid, param: PinJointParam) -> f32 {
        self.implementation.pin_joint_get_param(joint, param)
    }

    fn damped_spring_joint_set_param(&mut self, joint: Rid, param: DampedSpringParam, value: f32) {
        self.implementation
            .damped_spring_joint_set_param(joint, param, value);
    }

    fn damped_spring_joint_get_param(&self, joint: Rid, param: DampedSpringParam) -> f32 {
        self.implementation
            .damped_spring_joint_get_param(joint, param)
    }

    fn joint_get_type(&self, joint: Rid) -> JointType {
        self.implementation.joint_get_type(joint)
    }

    fn free_rid(&mut self, rid: Rid) {
        self.implementation.free_rid(rid);
    }

    fn set_active(&mut self, active: bool) {
        self.implementation.set_active(active);
    }

    fn init_ext(&mut self) {
        self.implementation.init_ext();
    }

    fn step(&mut self, step: f32) {
        self.implementation.step(step);
    }

    fn sync(&mut self) {
        self.implementation.sync();
    }

    fn flush_queries(&mut self) {
        let physics_data = physics_data();
        self.implementation.flushing_queries = true;
        let mut queries = Vec::default();
        for space in physics_data.active_spaces.values() {
            if let Some(space) = physics_data.spaces.get_mut(space) {
                let query = space.get_queries(&mut physics_data.collision_objects);
                queries.extend(query);
            }
        }
        let guard = self.base_mut();
        for query in queries {
            // TODO optimize function calls copying data.
            // TODO optimize after these are called, the callbacks into direct state objects.
            query.0.callv(Array::from(query.1.as_slice()));
        }
        drop(guard);
        self.implementation.flushing_queries = false;
        for space in physics_data.active_spaces.values() {
            if let Some(space) = physics_data.spaces.get_mut(space) {
                space.update_after_queries(&mut physics_data.collision_objects);
            }
        }
    }

    fn end_sync(&mut self) {
        self.implementation.end_sync();
    }

    fn finish(&mut self) {
        self.implementation.finish();
    }

    fn is_flushing_queries(&self) -> bool {
        self.implementation.is_flushing_queries()
    }

    fn get_process_info(&mut self, process_info: ProcessInfo) -> i32 {
        self.implementation.get_process_info(process_info)
    }
}
