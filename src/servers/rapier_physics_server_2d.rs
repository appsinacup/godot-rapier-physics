use std::collections::{HashMap, HashSet};

use godot::{engine, prelude::*};
use godot::engine::{IPhysicsServer2DExtension};

#[derive(GodotClass)]
#[class(base=PhysicsServer2DExtension, init)]
pub struct RapierPhysicsServer2D {
    active_spaces: HashSet<Rid>,
    active: bool,
    flushing_queries: bool,
}

#[godot_api]
impl IPhysicsServer2DExtension for RapierPhysicsServer2D {
    fn world_boundary_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn separation_ray_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn segment_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn circle_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn rectangle_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn capsule_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn convex_polygon_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn concave_polygon_shape_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn shape_set_data(&mut self, shape: Rid, data: Variant,) {
    }
    fn shape_set_custom_solver_bias(&mut self, shape: Rid, bias: f32,) {
    }
    fn shape_get_type(&self, shape: Rid,) -> engine::physics_server_2d::ShapeType {
        engine::physics_server_2d::ShapeType::CUSTOM
    }
    fn shape_get_data(&self, shape: Rid,) -> Variant {
        Variant::nil()
    }
    fn shape_get_custom_solver_bias(&self, shape: Rid,) -> f32 {
        0.0
    }
    fn space_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn space_set_active(&mut self, space: Rid, active: bool,) {
    }
    fn space_is_active(&self, space: Rid,) -> bool {
        false
    }
    fn space_set_param(&mut self, space: Rid, param: engine::physics_server_2d::SpaceParameter, value: f32,) {
    }
    fn space_get_param(&self, space: Rid, param: engine::physics_server_2d::SpaceParameter,) -> f32 {
        0.0
    }
    fn space_get_direct_state(&mut self, space: Rid,) -> Option < Gd < engine::PhysicsDirectSpaceState2D > > {
        None
    }
    fn space_set_debug_contacts(&mut self, space: Rid, max_contacts: i32,) {
    }
    fn space_get_contacts(&self, space: Rid,) -> PackedVector2Array {
        PackedVector2Array::new()
    }
    fn space_get_contact_count(&self, space: Rid,) -> i32 {
        0
    }
    fn area_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn area_set_space(&mut self, area: Rid, space: Rid,) {
    }
    fn area_get_space(&self, area: Rid,) -> Rid {
        Rid::Invalid
    }
    fn area_add_shape(&mut self, area: Rid, shape: Rid, transform: Transform2D, disabled: bool,) {
    }
    fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid,) {
    }
    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform2D,) {
    }
    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool,) {
    }
    fn area_get_shape_count(&self, area: Rid,) -> i32 {
        0
    }
    fn area_get_shape(&self, area: Rid, shape_idx: i32,) -> Rid {
        Rid::Invalid
    }
    fn area_get_shape_transform(&self, area: Rid, shape_idx: i32,) -> Transform2D {
        Transform2D::default()
    }
    fn area_remove_shape(&mut self, area: Rid, shape_idx: i32,) {
    }
    fn area_clear_shapes(&mut self, area: Rid,) {
    }
    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64,) {
    }
    fn area_get_object_instance_id(&self, area: Rid,) -> u64 {
        0
    }
    fn area_attach_canvas_instance_id(&mut self, area: Rid, id: u64,) {
    }
    fn area_get_canvas_instance_id(&self, area: Rid,) -> u64 {
        0
    }
    fn area_set_param(&mut self, area: Rid, param: engine::physics_server_2d::AreaParameter, value: Variant,) {
    }
    fn area_set_transform(&mut self, area: Rid, transform: Transform2D,) {
    }
    fn area_get_param(&self, area: Rid, param: engine::physics_server_2d::AreaParameter,) -> Variant {
        Variant::nil()
    }
    fn area_get_transform(&self, area: Rid,) -> Transform2D {
        Transform2D::default()
    }
    fn area_set_collision_layer(&mut self, area: Rid, layer: u32,) {
    }
    fn area_get_collision_layer(&self, area: Rid,) -> u32 {
        0
    }
    fn area_set_collision_mask(&mut self, area: Rid, mask: u32,) {
    }
    fn area_get_collision_mask(&self, area: Rid,) -> u32 {
        0
    }
    fn area_set_monitorable(&mut self, area: Rid, monitorable: bool,) {
    }
    fn area_set_pickable(&mut self, area: Rid, pickable: bool,) {
    }
    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable,) {
    }
    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable,) {
    }
    fn body_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn body_set_space(&mut self, body: Rid, space: Rid,) {
    }
    fn body_get_space(&self, body: Rid,) -> Rid {
        Rid::Invalid
    }
    fn body_set_mode(&mut self, body: Rid, mode: engine::physics_server_2d::BodyMode,) {
    }
    fn body_get_mode(&self, body: Rid,) -> engine::physics_server_2d::BodyMode {
        engine::physics_server_2d::BodyMode::KINEMATIC
    }
    fn body_add_shape(&mut self, body: Rid, shape: Rid, transform: Transform2D, disabled: bool,) {
    }
    fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid,) {
    }
    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform2D,) {
    }
    fn body_get_shape_count(&self, body: Rid,) -> i32 {
        0
    }
    fn body_get_shape(&self, body: Rid, shape_idx: i32,) -> Rid {
        Rid::Invalid
    }
    fn body_get_shape_transform(&self, body: Rid, shape_idx: i32,) -> Transform2D {
        Transform2D::default()
    }
    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool,) {
    }
    fn body_set_shape_as_one_way_collision(&mut self, body: Rid, shape_idx: i32, enable: bool, margin: f32,) {
    }
    fn body_remove_shape(&mut self, body: Rid, shape_idx: i32,) {
    }
    fn body_clear_shapes(&mut self, body: Rid,) {
    }
    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64,) {
    }
    fn body_get_object_instance_id(&self, body: Rid,) -> u64 {
        0
    }
    fn body_attach_canvas_instance_id(&mut self, body: Rid, id: u64,) {
    }
    fn body_get_canvas_instance_id(&self, body: Rid,) -> u64 {
        0
    }
    fn body_set_continuous_collision_detection_mode(&mut self, body: Rid, mode: engine::physics_server_2d::CcdMode,) {
    }
    fn body_get_continuous_collision_detection_mode(&self, body: Rid,) -> engine::physics_server_2d::CcdMode {
        engine::physics_server_2d::CcdMode::DISABLED
    }
    fn body_set_collision_layer(&mut self, body: Rid, layer: u32,) {
    }
    fn body_get_collision_layer(&self, body: Rid,) -> u32 {
        0
    }
    fn body_set_collision_mask(&mut self, body: Rid, mask: u32,) {
    }
    fn body_get_collision_mask(&self, body: Rid,) -> u32 {
        0
    }
    fn body_set_collision_priority(&mut self, body: Rid, priority: f32,) {
    }
    fn body_get_collision_priority(&self, body: Rid,) -> f32 {
        0.0
    }
    fn body_set_param(&mut self, body: Rid, param: engine::physics_server_2d::BodyParameter, value: Variant,) {
    }
    fn body_get_param(&self, body: Rid, param: engine::physics_server_2d::BodyParameter,) -> Variant {
        Variant::nil()
    }
    fn body_reset_mass_properties(&mut self, body: Rid,) {
    }
    fn body_set_state(&mut self, body: Rid, state: engine::physics_server_2d::BodyState, value: Variant,) {
    }
    fn body_get_state(&self, body: Rid, state: engine::physics_server_2d::BodyState,) -> Variant {
        Variant::nil()
    }
    fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector2,) {
    }
    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: f32,) {
    }
    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector2, position: Vector2,) {
    }
    fn body_apply_central_force(&mut self, body: Rid, force: Vector2,) {
    }
    fn body_apply_force(&mut self, body: Rid, force: Vector2, position: Vector2,) {
    }
    fn body_apply_torque(&mut self, body: Rid, torque: f32,) {
    }
    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector2,) {
    }
    fn body_add_constant_force(&mut self, body: Rid, force: Vector2, position: Vector2,) {
    }
    fn body_add_constant_torque(&mut self, body: Rid, torque: f32,) {
    }
    fn body_set_constant_force(&mut self, body: Rid, force: Vector2,) {
    }
    fn body_get_constant_force(&self, body: Rid,) -> Vector2 {
        Vector2::default()
    }
    fn body_set_constant_torque(&mut self, body: Rid, torque: f32,) {
    }
    fn body_get_constant_torque(&self, body: Rid,) -> f32 {
        0.0
    }
    fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector2,) {
    }
    fn body_add_collision_exception(&mut self, body: Rid, excepted_body: Rid,) {
    }
    fn body_remove_collision_exception(&mut self, body: Rid, excepted_body: Rid,) {
    }
    fn body_get_collision_exceptions(&self, body: Rid,) -> Array < Rid > {
        Array::new()
    }
    fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32,) {
    }
    fn body_get_max_contacts_reported(&self, body: Rid,) -> i32 {
        0
    }
    fn body_set_contacts_reported_depth_threshold(&mut self, body: Rid, threshold: f32,) {
    }
    fn body_get_contacts_reported_depth_threshold(&self, body: Rid,) -> f32 {
        0.0
    }
    fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool,) {
    }
    fn body_is_omitting_force_integration(&self, body: Rid,) -> bool {
        false
    }
    fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable,) {
    }
    fn body_set_force_integration_callback(&mut self, body: Rid, callable: Callable, userdata: Variant,) {
    }
    fn body_set_pickable(&mut self, body: Rid, pickable: bool,) {
    }
    fn body_get_direct_state(&mut self, body: Rid,) -> Option < Gd < engine::PhysicsDirectBodyState2D > > {
        None
    }
    fn joint_create(&mut self,) -> Rid {
        Rid::Invalid
    }
    fn joint_clear(&mut self, joint: Rid,) {
    }
    fn joint_set_param(&mut self, joint: Rid, param: engine::physics_server_2d::JointParam, value: f32,) {
    }
    fn joint_get_param(&self, joint: Rid, param: engine::physics_server_2d::JointParam,) -> f32 {
        0.0
    }
    fn joint_disable_collisions_between_bodies(&mut self, joint: Rid, disable: bool,) {
    }
    fn joint_is_disabled_collisions_between_bodies(&self, joint: Rid,) -> bool {
        false
    }
    fn joint_make_pin(&mut self, joint: Rid, anchor: Vector2, body_a: Rid, body_b: Rid,) {
    }
    fn joint_make_groove(&mut self, joint: Rid, a_groove1: Vector2, a_groove2: Vector2, b_anchor: Vector2, body_a: Rid, body_b: Rid,) {
    }
    fn joint_make_damped_spring(&mut self, joint: Rid, anchor_a: Vector2, anchor_b: Vector2, body_a: Rid, body_b: Rid,) {
    }
    fn pin_joint_set_flag(&mut self, joint: Rid, flag: engine::physics_server_2d::PinJointFlag, enabled: bool,) {
    }
    fn pin_joint_get_flag(&self, joint: Rid, flag: engine::physics_server_2d::PinJointFlag,) -> bool {
        false
    }
    fn pin_joint_set_param(&mut self, joint: Rid, param: engine::physics_server_2d::PinJointParam, value: f32,) {
    }
    fn pin_joint_get_param(&self, joint: Rid, param: engine::physics_server_2d::PinJointParam,) -> f32 {
        0.0
    }
    fn damped_spring_joint_set_param(&mut self, joint: Rid, param: engine::physics_server_2d::DampedSpringParam, value: f32,) {
    }
    fn damped_spring_joint_get_param(&self, joint: Rid, param: engine::physics_server_2d::DampedSpringParam,) -> f32 {
        0.0
    }
    fn joint_get_type(&self, joint: Rid,) -> engine::physics_server_2d::JointType {
        engine::physics_server_2d::JointType::MAX
    }
    fn free_rid(&mut self, rid: Rid,) {
    }
    fn set_active(&mut self, active: bool,) {
    }
    fn init_ext(&mut self,) {
    }
    fn step(&mut self, step: f32,) {
    }
    fn sync(&mut self,) {
    }
    fn flush_queries(&mut self,) {
    }
    fn end_sync(&mut self,) {
    }
    fn finish(&mut self,) {
    }
    fn is_flushing_queries(&self,) -> bool {
        false
    }
    fn get_process_info(&mut self, process_info: engine::physics_server_2d::ProcessInfo,) -> i32 {
        0
    }
}