use godot::classes::IPhysicsServer3DExtension;
use godot::classes::PhysicsServer3DExtension;
use godot::classes::ProjectSettings;
use godot::classes::{self};
use godot::engine::utilities::rid_allocate_id;
use godot::engine::utilities::rid_from_int64;
use godot::prelude::*;

use super::rapier_physics_singleton::active_spaces_singleton;
use super::rapier_physics_singleton::bodies_singleton;
use super::rapier_physics_singleton::fluids_singleton;
use super::rapier_physics_singleton::joints_singleton;
use super::rapier_physics_singleton::shapes_singleton;
use super::rapier_physics_singleton::spaces_singleton;
use super::rapier_project_settings::RapierProjectSettings;
use crate::bodies::rapier_area::RapierArea;
use crate::bodies::rapier_body::RapierBody;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::vector_normalized;
use crate::joints::rapier_joint::IRapierJoint;
use crate::joints::rapier_joint::RapierEmptyJoint;
use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_capsule_shape::RapierCapsuleShape;
use crate::shapes::rapier_circle_shape::RapierCircleShape;
use crate::shapes::rapier_convex_polygon_shape::RapierConvexPolygonShape;
use crate::shapes::rapier_cylinder_shape::RapierCylinderShape;
use crate::shapes::rapier_rectangle_shape::RapierRectangleShape;
use crate::shapes::rapier_shape::RapierShapeBase;
use crate::shapes::rapier_world_boundary_shape::RapierWorldBoundaryShape;
use crate::spaces::rapier_space::RapierSpace;
use crate::PackedVectorArray;
use crate::ANGLE_ZERO;
#[derive(GodotClass)]
#[class(base=PhysicsServer3DExtension, tool)]
pub struct RapierPhysicsServer3D {
    active: bool,
    flushing_queries: bool,
    doing_sync: bool,
    island_count: i32,
    active_objects: i32,
    collision_pairs: i32,

    base: Base<PhysicsServer3DExtension>,
}
#[godot_api]
impl IPhysicsServer3DExtension for RapierPhysicsServer3D {
    fn init(base: Base<PhysicsServer3DExtension>) -> Self {
        Self {
            active: true,
            flushing_queries: false,
            doing_sync: false,
            island_count: 0,
            active_objects: 0,
            collision_pairs: 0,
            base,
        }
    }

    fn world_boundary_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierWorldBoundaryShape::new(rid);
        shapes_singleton().shapes.insert(rid, Box::new(shape));
        rid
    }

    fn separation_ray_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierWorldBoundaryShape::new(rid);
        shapes_singleton().shapes.insert(rid, Box::new(shape));
        rid
    }

    fn sphere_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierCircleShape::new(rid);
        shapes_singleton().shapes.insert(rid, Box::new(shape));
        rid
    }

    fn box_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierRectangleShape::new(rid);
        shapes_singleton().shapes.insert(rid, Box::new(shape));
        rid
    }

    fn capsule_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierCapsuleShape::new(rid);
        shapes_singleton().shapes.insert(rid, Box::new(shape));
        rid
    }

    fn cylinder_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierCylinderShape::new(rid);
        shapes_singleton().shapes.insert(rid, Box::new(shape));
        rid
    }

    fn convex_polygon_shape_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let shape = RapierConvexPolygonShape::new(rid);
        shapes_singleton().shapes.insert(rid, Box::new(shape));
        rid
    }

    fn concave_polygon_shape_create(&mut self) -> Rid {
        Rid::Invalid
    }

    fn heightmap_shape_create(&mut self) -> Rid {
        Rid::Invalid
    }

    fn custom_shape_create(&mut self) -> Rid {
        Rid::Invalid
    }

    fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        let mut owners = None;
        if let Some(shape) = shapes_singleton().shapes.get_mut(&shape) {
            shape.set_data(data);
            if shape.get_handle().is_valid() {
                owners = Some(shape.get_base().get_owners().clone());
            }
        }
        if let Some(owners) = owners {
            RapierShapeBase::call_shape_changed(owners, shape);
        }
    }

    fn shape_set_custom_solver_bias(&mut self, _shape: Rid, _bias: f32) {}

    fn shape_get_type(&self, shape: Rid) -> classes::physics_server_3d::ShapeType {
        if let Some(shape) = shapes_singleton().shapes.get(&shape) {
            return shape.get_type();
        }
        classes::physics_server_3d::ShapeType::CUSTOM
    }

    fn shape_get_data(&self, shape: Rid) -> Variant {
        if let Some(shape) = shapes_singleton().shapes.get(&shape) {
            if shape.get_base().get_handle().is_valid() {
                return shape.get_data();
            }
        }
        Variant::nil()
    }

    fn shape_get_custom_solver_bias(&self, _shape: Rid) -> f32 {
        0.0
    }

    fn space_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let space = RapierSpace::new(rid);
        spaces_singleton().spaces.insert(rid, Box::new(space));
        rid
    }

    fn space_set_active(&mut self, space_rid: Rid, active: bool) {
        if let Some(space) = spaces_singleton().spaces.get(&space_rid) {
            if active {
                active_spaces_singleton()
                    .active_spaces
                    .insert(space.get_handle(), space_rid);
            }
            else {
                active_spaces_singleton()
                    .active_spaces
                    .remove(&space.get_handle());
            }
        }
    }

    fn space_is_active(&self, space: Rid) -> bool {
        if let Some(space) = spaces_singleton().spaces.get(&space) {
            return active_spaces_singleton()
                .active_spaces
                .contains_key(&space.get_handle());
        }
        false
    }

    fn space_set_param(
        &mut self,
        _space: Rid,
        _param: classes::physics_server_3d::SpaceParameter,
        _value: f32,
    ) {
    }

    fn space_get_param(
        &self,
        _space: Rid,
        _param: classes::physics_server_3d::SpaceParameter,
    ) -> f32 {
        0.0
    }

    fn space_get_direct_state(
        &mut self,
        space: Rid,
    ) -> Option<Gd<classes::PhysicsDirectSpaceState3D>> {
        if let Some(space) = spaces_singleton().spaces.get(&space) {
            return space.get_direct_state().clone();
        }
        None
    }

    fn space_set_debug_contacts(&mut self, space: Rid, max_contacts: i32) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&space) {
            space.set_debug_contacts(max_contacts);
        }
    }

    fn space_get_contacts(&self, space: Rid) -> PackedVectorArray {
        if let Some(space) = spaces_singleton().spaces.get(&space) {
            return space.get_debug_contacts().clone();
        }
        PackedVectorArray::new()
    }

    fn space_get_contact_count(&self, space: Rid) -> i32 {
        if let Some(space) = spaces_singleton().spaces.get(&space) {
            return space.get_debug_contact_count();
        }
        0
    }

    fn area_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let area = RapierArea::new(rid);
        bodies_singleton()
            .collision_objects
            .insert(rid, Box::new(area));
        rid
    }

    fn area_set_space(&mut self, area: Rid, space: Rid) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.set_space(space);
        }
    }

    fn area_get_space(&self, area: Rid) -> Rid {
        if let Some(area) = bodies_singleton().collision_objects.get(&area) {
            return area.get_base().get_space();
        }
        Rid::Invalid
    }

    fn area_add_shape(&mut self, area: Rid, shape: Rid, transform: Transform3D, disabled: bool) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.add_shape(shape, transform, disabled);
        }
    }

    fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.set_shape(shape_idx as usize, shape);
        }
    }

    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform3D) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.set_shape_transform(shape_idx as usize, transform);
        }
    }

    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.set_shape_disabled(shape_idx as usize, disabled);
        }
    }

    fn area_get_shape_count(&self, area: Rid) -> i32 {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            return area.get_base().get_shape_count();
        }
        -1
    }

    fn area_get_shape(&self, area: Rid, shape_idx: i32) -> Rid {
        if let Some(area) = bodies_singleton().collision_objects.get(&area) {
            return area.get_base().get_shape(shape_idx as usize);
        }
        Rid::Invalid
    }

    fn area_get_shape_transform(&self, area: Rid, shape_idx: i32) -> Transform3D {
        if let Some(area) = bodies_singleton().collision_objects.get(&area) {
            return area.get_base().get_shape_transform(shape_idx as usize);
        }
        Transform3D::default()
    }

    fn area_remove_shape(&mut self, area: Rid, shape_idx: i32) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.remove_shape_idx(shape_idx as usize);
        }
    }

    fn area_clear_shapes(&mut self, area: Rid) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            while area.get_base().get_shape_count() > 0 {
                area.remove_shape_idx(0);
            }
        }
    }

    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.get_mut_base().set_instance_id(id);
        }
    }

    fn area_get_object_instance_id(&self, area: Rid) -> u64 {
        if let Some(area) = bodies_singleton().collision_objects.get(&area) {
            return area.get_base().get_instance_id();
        }
        0
    }

    fn area_set_param(
        &mut self,
        area: Rid,
        param: classes::physics_server_3d::AreaParameter,
        value: Variant,
    ) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&area) {
            space.set_default_area_param(param, value);
            return;
        }
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            if let Some(area) = area.get_mut_area() {
                area.set_param(param, value);
            }
        }
    }

    fn area_set_transform(&mut self, area: Rid, transform: Transform3D) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.get_mut_base().set_transform(transform, false);
        }
    }

    fn area_get_param(
        &self,
        area: Rid,
        param: classes::physics_server_3d::AreaParameter,
    ) -> Variant {
        if let Some(space) = spaces_singleton().spaces.get_mut(&area) {
            return space.get_default_area_param(param);
        }
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            if let Some(area) = area.get_mut_area() {
                return area.get_param(param);
            }
        }
        Variant::nil()
    }

    fn area_get_transform(&self, area: Rid) -> Transform3D {
        if let Some(area) = bodies_singleton().collision_objects.get(&area) {
            return area.get_base().get_transform();
        }
        Transform3D::default()
    }

    fn area_set_collision_layer(&mut self, area: Rid, layer: u32) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.get_mut_base().set_collision_layer(layer);
        }
    }

    fn area_get_collision_layer(&self, area: Rid) -> u32 {
        if let Some(area) = bodies_singleton().collision_objects.get(&area) {
            return area.get_base().get_collision_layer();
        }
        0
    }

    fn area_set_collision_mask(&mut self, area: Rid, mask: u32) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.get_mut_base().set_collision_mask(mask);
        }
    }

    fn area_get_collision_mask(&self, area: Rid) -> u32 {
        if let Some(area) = bodies_singleton().collision_objects.get(&area) {
            return area.get_base().get_collision_mask();
        }
        0
    }

    fn area_set_monitorable(&mut self, area: Rid, monitorable: bool) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            if let Some(area) = area.get_mut_area() {
                area.set_monitorable(monitorable);
            }
        }
    }

    fn area_set_ray_pickable(&mut self, area: Rid, enable: bool) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            area.get_mut_base().set_pickable(enable);
        }
    }

    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            if let Some(area) = area.get_mut_area() {
                area.set_monitor_callback(callback);
            }
        }
    }

    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
            if let Some(area) = area.get_mut_area() {
                area.set_area_monitor_callback(callback);
            }
        }
    }

    fn body_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        let body = RapierBody::new(rid);
        bodies_singleton()
            .collision_objects
            .insert(rid, Box::new(body));
        rid
    }

    fn body_set_space(&mut self, body: Rid, space: Rid) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            body.set_space(space);
        }
    }

    fn body_get_space(&self, body: Rid) -> Rid {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            return body.get_base().get_space();
        }
        Rid::Invalid
    }

    fn body_set_mode(&mut self, body: Rid, mode: classes::physics_server_3d::BodyMode) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_mode(mode);
            }
        }
    }

    fn body_get_mode(&self, body: Rid) -> classes::physics_server_3d::BodyMode {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_base().get_mode();
            }
        }
        classes::physics_server_3d::BodyMode::STATIC
    }

    fn body_add_shape(&mut self, body: Rid, shape: Rid, transform: Transform3D, disabled: bool) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.add_shape(shape, transform, disabled);
            }
        }
    }

    fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_shape(shape_idx as usize, shape);
            }
        }
    }

    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform3D) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_shape_transform(shape_idx as usize, transform);
            }
        }
    }

    fn body_get_shape_count(&self, body: Rid) -> i32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_base().get_shape_count();
            }
        }
        0
    }

    fn body_get_shape(&self, body: Rid, shape_idx: i32) -> Rid {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_base().get_shape(shape_idx as usize);
            }
        }
        Rid::Invalid
    }

    fn body_get_shape_transform(&self, body: Rid, shape_idx: i32) -> Transform3D {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_base().get_shape_transform(shape_idx as usize);
            }
        }
        Transform3D::default()
    }

    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_shape_disabled(shape_idx as usize, disabled);
            }
        }
    }

    fn body_remove_shape(&mut self, body: Rid, shape_idx: i32) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.remove_shape_idx(shape_idx as usize);
            }
        }
    }

    fn body_clear_shapes(&mut self, body: Rid) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            while body.get_base().get_shape_count() > 0 {
                body.remove_shape_idx(0);
            }
        }
    }

    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            body.get_mut_base().set_instance_id(id);
        }
    }

    fn body_get_object_instance_id(&self, body: Rid) -> u64 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            return body.get_base().get_instance_id();
        }
        0
    }

    fn body_set_collision_layer(&mut self, body: Rid, layer: u32) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            body.get_mut_base().set_collision_layer(layer);
        }
    }

    fn body_get_collision_layer(&self, body: Rid) -> u32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            return body.get_base().get_collision_layer();
        }
        0
    }

    fn body_set_collision_mask(&mut self, body: Rid, mask: u32) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            body.get_mut_base().set_collision_mask(mask);
        }
    }

    fn body_get_collision_mask(&self, body: Rid) -> u32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            return body.get_base().get_collision_mask();
        }
        0
    }

    fn body_set_collision_priority(&mut self, body: Rid, priority: f32) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            body.get_mut_base().set_collision_priority(priority);
        }
    }

    fn body_get_collision_priority(&self, body: Rid) -> f32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            return body.get_base().get_collision_priority();
        }
        0.0
    }

    fn body_set_param(
        &mut self,
        body: Rid,
        param: classes::physics_server_3d::BodyParameter,
        value: Variant,
    ) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_param(param, value);
            }
        }
    }

    fn body_get_param(
        &self,
        body: Rid,
        param: classes::physics_server_3d::BodyParameter,
    ) -> Variant {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_param(param);
            }
        }
        Variant::nil()
    }

    fn body_reset_mass_properties(&mut self, body: Rid) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.reset_mass_properties();
            }
        }
    }

    fn body_set_state(
        &mut self,
        body: Rid,
        state: classes::physics_server_3d::BodyState,
        value: Variant,
    ) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_state(state, value);
            }
        }
    }

    fn body_get_state(&self, body: Rid, state: classes::physics_server_3d::BodyState) -> Variant {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_state(state);
            }
        }
        Variant::nil()
    }

    fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_central_impulse(impulse);
            }
        }
    }

    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_torque_impulse(impulse);
            }
        }
    }

    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector3, position: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_impulse(impulse, position);
            }
        }
    }

    fn body_apply_central_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_central_force(force);
            }
        }
    }

    fn body_apply_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_force(force, position);
            }
        }
    }

    fn body_apply_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.apply_torque(torque);
            }
        }
    }

    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.add_constant_central_force(force);
            }
        }
    }

    fn body_add_constant_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.add_constant_force(force, position);
            }
        }
    }

    fn body_add_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.add_constant_torque(torque);
            }
        }
    }

    fn body_set_constant_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_constant_force(force);
            }
        }
    }

    fn body_get_constant_force(&self, body: Rid) -> Vector3 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_force();
            }
        }
        Vector3::default()
    }

    fn body_set_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_constant_torque(torque);
            }
        }
    }

    fn body_get_constant_torque(&self, body: Rid) -> Vector3 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_constant_torque();
            }
        }
        ANGLE_ZERO
    }

    fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector3) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                let mut v = body.get_linear_velocity();
                let axis = vector_normalized(axis_velocity);
                v -= axis * axis.dot(v);
                v += axis_velocity;
                body.set_linear_velocity(v);
            }
        }
    }

    fn body_add_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.add_exception(excepted_body);
                body.wakeup();
            }
        }
    }

    fn body_remove_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.remove_exception(excepted_body);
                body.wakeup();
            }
        }
    }

    fn body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                let exceptions = body.get_exceptions();
                let mut arr = Array::new();
                for e in exceptions {
                    arr.push(*e);
                }
                return arr;
            }
        }
        Array::new()
    }

    fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_max_contacts_reported(amount);
            }
        }
    }

    fn body_get_max_contacts_reported(&self, body: Rid) -> i32 {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_max_contacts_reported();
            }
        }
        0
    }

    fn body_set_contacts_reported_depth_threshold(&mut self, _body: Rid, _threshold: f32) {}

    fn body_get_contacts_reported_depth_threshold(&self, _body: Rid) -> f32 {
        0.0
    }

    fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_omit_force_integration(enable);
            }
        }
    }

    fn body_is_omitting_force_integration(&self, body: Rid) -> bool {
        if let Some(body) = bodies_singleton().collision_objects.get(&body) {
            if let Some(body) = body.get_body() {
                return body.get_omit_force_integration();
            }
        }
        false
    }

    fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_state_sync_callback(callable);
            }
        }
    }

    fn body_set_force_integration_callback(
        &mut self,
        body: Rid,
        callable: Callable,
        userdata: Variant,
    ) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                body.set_force_integration_callback(callable, userdata);
            }
        }
    }

    fn body_set_ray_pickable(&mut self, body: Rid, enable: bool) {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            body.get_mut_base().set_pickable(enable);
        }
    }

    fn body_get_direct_state(
        &mut self,
        body: Rid,
    ) -> Option<Gd<classes::PhysicsDirectBodyState3D>> {
        if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
            if let Some(body) = body.get_mut_body() {
                return body.get_direct_state().cloned();
            }
        }
        None
    }

    fn joint_create(&mut self) -> Rid {
        let rid = rid_from_int64(rid_allocate_id());
        joints_singleton()
            .joints
            .insert(rid, Box::new(RapierEmptyJoint::new()));
        rid
    }

    fn joint_clear(&mut self, rid: Rid) {
        if let Some(prev_joint) = joints_singleton().joints.remove(&rid) {
            let mut joint = RapierEmptyJoint::new();
            joint
                .get_mut_base()
                .copy_settings_from(prev_joint.get_base());
            joints_singleton().joints.insert(rid, Box::new(joint));
        }
    }

    fn joint_disable_collisions_between_bodies(&mut self, joint: Rid, disable: bool) {
        if let Some(joint) = joints_singleton().joints.get_mut(&joint) {
            joint
                .get_mut_base()
                .disable_collisions_between_bodies(disable);
        }
    }

    fn joint_is_disabled_collisions_between_bodies(&self, joint: Rid) -> bool {
        if let Some(joint) = joints_singleton().joints.get(&joint) {
            return joint.get_base().is_disabled_collisions_between_bodies();
        }
        true
    }

    fn joint_get_type(&self, joint: Rid) -> classes::physics_server_3d::JointType {
        if let Some(joint) = joints_singleton().joints.get(&joint) {
            return joint.get_type();
        }
        classes::physics_server_3d::JointType::MAX
    }

    fn free_rid(&mut self, rid: Rid) {
        if let Some(shape) = shapes_singleton().shapes.remove(&rid) {
            for (owner, _) in shape.get_base().get_owners() {
                if let Some(body) = bodies_singleton().collision_objects.get_mut(owner) {
                    body.remove_shape_rid(shape.get_base().get_rid());
                }
            }
            return;
        }
        if let Some(mut body) = bodies_singleton().collision_objects.remove(&rid) {
            body.set_space(Rid::Invalid);
            while body.get_base().get_shape_count() > 0 {
                body.remove_shape_idx(0);
            }
            return;
        }
        if let Some(space) = spaces_singleton().spaces.remove(&rid) {
            let space_handle = space.get_handle();
            if active_spaces_singleton()
                .active_spaces
                .remove(&space_handle)
                .is_some()
            {
                return;
            }
        }
        if let Some(_joint) = joints_singleton().joints.remove(&rid) {
            return;
        }
        if let Some(_fluid) = fluids_singleton().fluids.remove(&rid) {}
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
        for space_rid in active_spaces_singleton().active_spaces.values() {
            let body_area_update_list;
            let gravity_update_list;
            let space_handle;
            if let Some(space) = spaces_singleton().spaces.get_mut(space_rid) {
                body_area_update_list = space.get_body_area_update_list().clone();
                gravity_update_list = space.get_gravity_update_list().clone();
                space_handle = space.get_handle();
                space.before_step();
                for body in space.get_active_list() {
                    if let Some(body) = bodies_singleton().collision_objects.get_mut(body) {
                        if let Some(body) = body.get_mut_body() {
                            body.reset_contact_count();
                        }
                    }
                }
                for body in space.get_mass_properties_update_list() {
                    if let Some(body) = bodies_singleton().collision_objects.get_mut(body) {
                        if let Some(body) = body.get_mut_body() {
                            body.update_mass_properties(false);
                        }
                    }
                }
                for area in space.get_area_update_list().clone() {
                    if let Some(area) = bodies_singleton().collision_objects.get_mut(&area) {
                        if let Some(area) = area.get_mut_area() {
                            area.update_area_override(space);
                        }
                    }
                }
            }
            else {
                continue;
            }
            for body in body_area_update_list {
                if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
                    if let Some(body) = body.get_mut_body() {
                        body.update_area_override();
                    }
                }
            }
            for body in gravity_update_list {
                if let Some(body) = bodies_singleton().collision_objects.get_mut(&body) {
                    if let Some(body) = body.get_mut_body() {
                        body.update_gravity(step);
                    }
                }
            }
            let project_settings = ProjectSettings::singleton();
            let default_gravity_dir: Vector3 = project_settings
                .get_setting_with_override("physics/3d/default_gravity_vector".into())
                .to();
            let default_gravity_value: real = project_settings
                .get_setting_with_override("physics/3d/default_gravity".into())
                .to();
            //let default_linear_damping: real = project_settings.get_setting_with_override("physics/2d/default_linear_damp".into()).to();
            //let default_angular_damping: real = project_settings.get_setting_with_override("physics/2d/default_angular_damp".into()).to();
            let settings = SimulationSettings {
                pixel_liquid_gravity: vector_to_rapier(default_gravity_dir) * default_gravity_value,
                dt: step,
                length_unit: RapierProjectSettings::get_length_unit(),
                pixel_gravity: vector_to_rapier(default_gravity_dir) * default_gravity_value,
                max_ccd_substeps: RapierProjectSettings::get_solver_max_ccd_substeps() as usize,
                num_additional_friction_iterations:
                    RapierProjectSettings::get_solver_num_additional_friction_iterations() as usize,
                num_internal_pgs_iterations:
                    RapierProjectSettings::get_solver_num_internal_pgs_iterations() as usize,
                num_solver_iterations: RapierProjectSettings::get_solver_num_solver_iterations()
                    as usize,
            };
            if let Some(space) = spaces_singleton().spaces.get_mut(space_rid) {
                // this calls into rapier
                world_step(
                    space_handle,
                    &settings,
                    RapierSpace::active_body_callback,
                    RapierSpace::collision_filter_body_callback,
                    RapierSpace::collision_filter_sensor_callback,
                    RapierSpace::collision_modify_contacts_callback,
                    RapierSpace::collision_event_callback,
                    RapierSpace::contact_force_event_callback,
                    RapierSpace::contact_point_callback,
                    space,
                );
                space.after_step();
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
        let active_spaces = active_spaces_singleton().active_spaces.clone();
        for space in active_spaces.values() {
            if let Some(space) = spaces_singleton().spaces.get_mut(space) {
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

    fn get_process_info(&mut self, process_info: classes::physics_server_3d::ProcessInfo) -> i32 {
        match process_info {
            classes::physics_server_3d::ProcessInfo::ACTIVE_OBJECTS => self.active_objects,
            classes::physics_server_3d::ProcessInfo::COLLISION_PAIRS => self.collision_pairs,
            classes::physics_server_3d::ProcessInfo::ISLAND_COUNT => self.island_count,
            _ => 0,
        }
    }
}
