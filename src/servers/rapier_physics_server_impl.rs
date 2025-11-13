#[cfg(feature = "dim2")]
use std::ffi::c_void;

#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::global::rid_allocate_id;
use godot::global::rid_from_int64;
use godot::prelude::*;

use super::rapier_physics_singleton::RapierId;
use super::rapier_physics_singleton::get_id_rid;
use super::rapier_physics_singleton::insert_id_rid;
use super::rapier_physics_singleton::physics_data;
use super::rapier_physics_singleton::remove_id_rid;
use super::rapier_project_settings::RapierProjectSettings;
use crate::bodies::rapier_area::AreaUpdateMode;
use crate::bodies::rapier_area::RapierArea;
use crate::bodies::rapier_body::RapierBody;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
#[cfg(feature = "dim2")]
use crate::joints::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
use crate::joints::rapier_empty_joint::RapierEmptyJoint;
#[cfg(feature = "dim2")]
use crate::joints::rapier_groove_joint_2d::RapierGrooveJoint2D;
use crate::joints::rapier_joint::IRapierJoint;
use crate::joints::rapier_joint::RapierJoint;
use crate::joints::rapier_joint_base::RapierJointType;
use crate::joints::rapier_revolute_joint::RapierRevoluteJoint;
#[cfg(feature = "dim3")]
use crate::joints::rapier_spherical_joint_3d::RapierSphericalJoint3D;
use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_capsule_shape::RapierCapsuleShape;
use crate::shapes::rapier_circle_shape::RapierCircleShape;
use crate::shapes::rapier_concave_polygon_shape::RapierConcavePolygonShape;
use crate::shapes::rapier_convex_polygon_shape::RapierConvexPolygonShape;
#[cfg(feature = "dim3")]
use crate::shapes::rapier_cylinder_shape_3d::RapierCylinderShape3D;
#[cfg(feature = "dim3")]
use crate::shapes::rapier_heightmap_shape_3d::RapierHeightMapShape3D;
use crate::shapes::rapier_rectangle_shape::RapierRectangleShape;
#[cfg(feature = "dim2")]
use crate::shapes::rapier_segment_shape_2d::RapierSegmentShape2D;
use crate::shapes::rapier_separation_ray_shape::RapierSeparationRayShape;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
use crate::shapes::rapier_world_boundary_shape::RapierWorldBoundaryShape;
use crate::spaces::rapier_space::RapierSpace;
use crate::types::*;
pub struct RapierPhysicsServerImpl {
    pub id: RapierId,
    pub active: bool,
    pub stepped: bool,
    pub flushing_queries: bool,
    doing_sync: bool,
    active_objects: i32,
    length_unit: real,
    max_ccd_substeps: usize,
    num_internal_pgs_iterations: usize,
    num_solver_iterations: usize,
    normalized_allowed_linear_error: real,
    normalized_max_corrective_velocity: real,
    normalized_prediction_distance: real,
    predictive_contact_allowance_threshold: real,
    num_internal_stabilization_iterations: usize,
    contact_damping_ratio: real,
    contact_natural_frequency: real,
}
impl RapierPhysicsServerImpl {
    pub(super) fn next_id(&mut self) -> RapierId {
        self.id += 1;
        self.id
    }

    pub(super) fn default() -> Self {
        Self {
            id: RapierId::default(),
            active: true,
            stepped: false,
            flushing_queries: false,
            doing_sync: false,
            active_objects: 0,
            length_unit: RapierProjectSettings::get_length_unit(),
            max_ccd_substeps: RapierProjectSettings::get_solver_max_ccd_substeps() as usize,
            num_internal_pgs_iterations:
                RapierProjectSettings::get_solver_num_internal_pgs_iterations() as usize,
            num_solver_iterations: RapierProjectSettings::get_solver_num_solver_iterations()
                as usize,
            normalized_allowed_linear_error:
                RapierProjectSettings::get_normalized_allowed_linear_error(),
            normalized_max_corrective_velocity:
                RapierProjectSettings::get_normalized_max_corrective_velocity(),
            normalized_prediction_distance:
                RapierProjectSettings::get_normalized_prediction_distance(),
            predictive_contact_allowance_threshold:
                RapierProjectSettings::get_predictive_contact_allowance_threshold(),
            num_internal_stabilization_iterations:
                RapierProjectSettings::get_num_internal_stabilization_iterations() as usize,
            contact_damping_ratio: RapierProjectSettings::get_contact_damping_ratio(),
            contact_natural_frequency: RapierProjectSettings::get_contact_natural_frequency(),
        }
    }

    pub(super) fn world_boundary_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierWorldBoundaryShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn separation_ray_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierSeparationRayShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    #[cfg(feature = "dim2")]
    pub(super) fn segment_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierSegmentShape2D::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn circle_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierCircleShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn rectangle_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierRectangleShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn capsule_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierCapsuleShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    #[cfg(feature = "dim3")]
    pub(super) fn cylinder_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierCylinderShape3D::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn convex_polygon_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierConvexPolygonShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    #[cfg(feature = "dim2")]
    pub(super) fn concave_polygon_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierConcavePolygonShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    #[cfg(feature = "dim3")]
    pub(super) fn concave_polygon_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierConcavePolygonShape::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    #[cfg(feature = "dim3")]
    pub(super) fn heightmap_shape_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierHeightMapShape3D::create(id, rid, &mut physics_data.shapes);
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        let physics_data = physics_data();
        let mut owners = None;
        let mut shape_id = RapierId::default();
        if let Some(shape) = physics_data.shapes.get_mut(&shape) {
            shape_id = shape.get_base().get_id();
            shape.set_data(data, &mut physics_data.physics_engine);
            owners = Some(shape.get_base().get_owners().clone());
        }
        if shape_id == RapierId::default() {
            godot_error!("Invalid shape id");
            return;
        }
        if let Some(owners) = owners {
            RapierShapeBase::call_shape_changed(owners, shape_id, physics_data);
        }
    }

    pub(super) fn shape_get_type(&self, shape: Rid) -> ShapeType {
        let physics_data = physics_data();
        if let Some(shape) = physics_data.shapes.get(&shape) {
            return shape.get_type();
        }
        ShapeType::CUSTOM
    }

    #[cfg(feature = "dim3")]
    pub(super) fn shape_set_margin(&mut self, _shape: Rid, _margin: real) {}

    #[cfg(feature = "dim3")]
    pub(super) fn shape_get_margin(&self, _shape: Rid) -> real {
        0.0
    }

    pub(super) fn shape_get_data(&self, shape: Rid) -> Variant {
        let physics_data = physics_data();
        if let Some(shape) = physics_data.shapes.get(&shape) {
            return shape.get_data(&physics_data.physics_engine);
        }
        Variant::nil()
    }

    pub(super) fn shape_get_custom_solver_bias(&self, _shape: Rid) -> f32 {
        0.0
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub(super) unsafe fn shape_collide(
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
        let physics_data = physics_data();
        let [shape_a, shape_b] = physics_data.shapes.get_many_mut([&shape_a, &shape_b]);
        let (Some(shape_a), Some(shape_b)) = (shape_a, shape_b) else {
            return false;
        };
        let shape_a_handle = shape_a.get_base().get_id();
        let shape_b_handle = shape_b.get_base().get_id();
        let shape_a_info = shape_info_from_body_shape(shape_a_handle, xform_a);
        let shape_b_info = shape_info_from_body_shape(shape_b_handle, xform_b);
        let rapier_a_motion = vector_to_rapier(motion_a);
        let rapier_b_motion = vector_to_rapier(motion_b);
        let results_out: *mut Vector = results as *mut Vector;
        let result = physics_data.physics_engine.shape_collide(
            rapier_a_motion,
            shape_a_info,
            rapier_b_motion,
            shape_b_info,
        );
        if !result.collided {
            return false;
        }
        if result_max >= 1 {
            unsafe { *result_count = 1 };
            let vector2_slice: &mut [Vector] =
                unsafe { std::slice::from_raw_parts_mut(results_out, result_max as usize) };
            vector2_slice[0] = vector_to_godot(result.pixel_witness1);
            vector2_slice[1] = vector_to_godot(result.pixel_witness2);
        }
        true
    }

    pub(super) fn space_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        RapierSpace::create(
            id,
            rid,
            &mut physics_data.physics_engine,
            &mut physics_data.spaces,
        );
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn space_set_active(&mut self, space_rid: Rid, active: bool) {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space_rid) {
            if active {
                physics_data
                    .active_spaces
                    .insert(space.get_state().get_id(), space_rid);
            } else {
                physics_data
                    .active_spaces
                    .remove(&space.get_state().get_id());
            }
        }
    }

    pub(super) fn space_is_active(&self, space: Rid) -> bool {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space) {
            return physics_data
                .active_spaces
                .contains_key(&space.get_state().get_id());
        }
        false
    }

    pub(super) fn space_set_param(&mut self, _space: Rid, _param: SpaceParameter, _value: f32) {}

    pub(super) fn space_get_param(&self, _space: Rid, _param: SpaceParameter) -> f32 {
        0.0
    }

    pub(super) fn space_get_direct_state(
        &mut self,
        space: Rid,
    ) -> Option<Gd<PhysicsDirectSpaceState>> {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space) {
            return space.get_direct_state().clone();
        }
        None
    }

    pub(super) fn space_set_debug_contacts(&mut self, space: Rid, max_contacts: i32) {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get_mut(&space) {
            space.set_debug_contacts(max_contacts);
        }
    }

    pub(super) fn space_get_contacts(&self, space: Rid) -> PackedVectorArray {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space) {
            return space.get_debug_contacts().clone();
        }
        PackedVectorArray::new()
    }

    pub(super) fn space_get_contact_count(&self, space: Rid) -> i32 {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&space) {
            return space.get_debug_contact_count();
        }
        0
    }

    pub(super) fn space_get_active_bodies(&self, space: Rid) -> Array<Rid> {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get_mut(&space) {
            let bodies = space.get_state().get_active_bodies();
            let mut array: Array<Rid> = Array::new();
            for body_id in bodies {
                let rid = get_id_rid(body_id, &physics_data.ids);
                array.push(rid);
            }
            return array;
        }
        Array::default()
    }

    pub(super) fn space_get_bodies_transform(
        &self,
        space: Rid,
        bodies: Array<Rid>,
    ) -> Array<Transform> {
        let physics_data = physics_data();
        let mut array = Array::default();
        if let Some(_space) = physics_data.spaces.get_mut(&space) {
            for body in bodies.iter_shared() {
                if let Some(body) = physics_data.collision_objects.get_mut(&body) {
                    let transform = body.get_base().get_transform();
                    array.push(transform);
                }
            }
        }
        array
    }

    pub(super) fn area_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        let area = RapierArea::new(id, rid);
        insert_id_rid(area.get_base().get_id(), rid, &mut physics_data.ids);
        physics_data
            .collision_objects
            .insert(rid, RapierCollisionObject::RapierArea(area));
        rid
    }

    pub(super) fn area_set_space(&mut self, area: Rid, space: Rid) {
        let physics_data = physics_data();
        RapierArea::clear_detected_bodies(
            &area,
            &mut physics_data.spaces,
            &mut physics_data.collision_objects,
            &physics_data.ids,
        );
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.set_space(
                space,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.ids,
            );
        }
    }

    pub(super) fn area_get_space(&self, area: Rid) -> Rid {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_space(&physics_data.ids);
        }
        Rid::Invalid
    }

    pub(super) fn area_add_shape(
        &mut self,
        area: Rid,
        shape: Rid,
        transform: Transform,
        disabled: bool,
    ) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area)
            && let Some(shape) = physics_data.shapes.get(&shape)
        {
            area.add_shape(
                shape.get_base().get_id(),
                transform,
                disabled,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.shapes,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area)
            && let Some(shape) = physics_data.shapes.get(&shape)
        {
            area.set_shape(
                shape_idx as usize,
                shape.get_base().get_id(),
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.shapes,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn area_set_shape_transform(
        &mut self,
        area: Rid,
        shape_idx: i32,
        transform: Transform,
    ) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.set_shape_transform(
                shape_idx as usize,
                transform,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.set_shape_disabled(
                shape_idx as usize,
                disabled,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn area_get_shape_count(&self, area: Rid) -> i32 {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_shape_count();
        }
        -1
    }

    pub(super) fn area_get_shape(&self, area: Rid, shape_idx: i32) -> Rid {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area
                .get_base()
                .get_shape(&physics_data.ids, shape_idx as usize);
        }
        Rid::Invalid
    }

    pub(super) fn area_get_shape_transform(&self, area: Rid, shape_idx: i32) -> Transform {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_shape_transform(shape_idx as usize);
        }
        Transform::IDENTITY
    }

    pub(super) fn area_remove_shape(&mut self, area: Rid, shape_idx: i32) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.remove_shape_idx(
                shape_idx as usize,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.shapes,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn area_clear_shapes(&mut self, area: Rid) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            while area.get_base().get_shape_count() > 0 {
                area.remove_shape_idx(
                    0,
                    &mut physics_data.physics_engine,
                    &mut physics_data.spaces,
                    &mut physics_data.shapes,
                    &physics_data.ids,
                );
            }
        }
    }

    pub(super) fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.get_mut_base().set_instance_id(id);
        }
    }

    pub(super) fn area_get_object_instance_id(&self, area: Rid) -> u64 {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_instance_id();
        }
        0
    }

    #[cfg(feature = "dim2")]
    pub(super) fn area_attach_canvas_instance_id(&mut self, area: Rid, id: u64) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.get_mut_base().set_canvas_instance_id(id);
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn area_get_canvas_instance_id(&self, area: Rid) -> u64 {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_canvas_instance_id();
        }
        0
    }

    pub(super) fn area_set_param(&mut self, area: Rid, param: AreaParameter, value: Variant) {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get_mut(&area) {
            space.set_default_area_param(param, value);
            return;
        }
        let area_update_mode = AreaUpdateMode::None;
        let mut area_id = RapierId::default();
        if let Some(area) = physics_data.collision_objects.get_mut(&area)
            && let Some(area) = area.get_mut_area()
        {
            area.set_param(param, value, &mut physics_data.spaces, &physics_data.ids);
            area_id = area.get_base().get_id();
        }
        if area_id == RapierId::default() {
            godot_error!("Area not found");
            return;
        }
        match area_update_mode {
            AreaUpdateMode::EnableSpaceOverride => {
                RapierArea::enable_space_override(
                    &area_id,
                    &mut physics_data.spaces,
                    &mut physics_data.collision_objects,
                    &physics_data.ids,
                );
            }
            AreaUpdateMode::DisableSpaceOverride => {
                RapierArea::disable_space_override(
                    &area_id,
                    &mut physics_data.spaces,
                    &mut physics_data.collision_objects,
                    &physics_data.ids,
                );
            }
            AreaUpdateMode::ResetSpaceOverride => {
                RapierArea::reset_space_override(
                    &area_id,
                    &mut physics_data.spaces,
                    &mut physics_data.collision_objects,
                    &physics_data.ids,
                );
            }
            _ => {}
        }
    }

    pub(super) fn area_set_transform(&mut self, area: Rid, transform: Transform) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.get_mut_base()
                .set_transform(transform, false, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn area_get_param(&self, area: Rid, param: AreaParameter) -> Variant {
        let physics_data = physics_data();
        if let Some(space) = physics_data.spaces.get(&area) {
            return space.get_default_area_param(param);
        }
        if let Some(area) = physics_data.collision_objects.get(&area)
            && let Some(area) = area.get_area()
        {
            return area.get_param(param);
        }
        Variant::nil()
    }

    pub(super) fn area_get_transform(&self, area: Rid) -> Transform {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_transform();
        }
        Transform::IDENTITY
    }

    pub(super) fn area_set_collision_layer(&mut self, area: Rid, layer: u32) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.get_mut_base()
                .set_collision_layer(layer, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn area_get_collision_layer(&self, area: Rid) -> u32 {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_collision_layer();
        }
        0
    }

    pub(super) fn area_set_collision_mask(&mut self, area: Rid, mask: u32) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.get_mut_base()
                .set_collision_mask(mask, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn area_get_collision_mask(&self, area: Rid) -> u32 {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get(&area) {
            return area.get_base().get_collision_mask();
        }
        0
    }

    pub(super) fn area_set_monitorable(&mut self, area: Rid, monitorable: bool) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area)
            && let Some(area) = area.get_mut_area()
        {
            area.set_monitorable(
                monitorable,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn area_set_pickable(&mut self, area: Rid, pickable: bool) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.get_mut_base().set_pickable(pickable);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn area_set_ray_pickable(&mut self, area: Rid, pickable: bool) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area) {
            area.get_mut_base().set_pickable(pickable);
        }
    }

    pub(super) fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area)
            && let Some(area) = area.get_mut_area()
        {
            area.set_monitor_callback(
                callback,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {
        let physics_data = physics_data();
        if let Some(area) = physics_data.collision_objects.get_mut(&area)
            && let Some(area) = area.get_mut_area()
        {
            area.set_area_monitor_callback(
                callback,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        let body = RapierBody::new(id, rid);
        insert_id_rid(body.get_base().get_id(), rid, &mut physics_data.ids);
        physics_data
            .collision_objects
            .insert(rid, RapierCollisionObject::RapierBody(body));
        rid
    }

    pub(super) fn body_set_space(&mut self, body: Rid, space: Rid) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.set_space(
                space,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.ids,
            );
        }
    }

    pub(super) fn body_get_space(&self, body: Rid) -> Rid {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body) {
            return body.get_base().get_space(&physics_data.ids);
        }
        Rid::Invalid
    }

    pub(super) fn body_set_mode(&mut self, body: Rid, mode: BodyMode) {
        let physics_data = physics_data();
        let mut body_id = RapierId::default();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_mode(
                mode,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
            body_id = body.get_base().get_id();
        }
        if body_id != RapierId::default() {
            RapierBody::apply_area_override_to_body(
                &body_id,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_get_mode(&self, body: Rid) -> BodyMode {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_base().get_mode();
        }
        BodyMode::STATIC
    }

    pub(super) fn body_add_shape(
        &mut self,
        body: Rid,
        shape: Rid,
        transform: Transform,
        disabled: bool,
    ) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
            && let Some(shape) = physics_data.shapes.get(&shape)
        {
            body.add_shape(
                shape.get_base().get_id(),
                transform,
                disabled,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.shapes,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
            && let Some(shape) = physics_data.shapes.get(&shape)
        {
            body.set_shape(
                shape_idx as usize,
                shape.get_base().get_id(),
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.shapes,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_set_shape_transform(
        &mut self,
        body: Rid,
        shape_idx: i32,
        transform: Transform,
    ) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_shape_transform(
                shape_idx as usize,
                transform,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_get_shape_count(&self, body: Rid) -> i32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_base().get_shape_count();
        }
        0
    }

    pub(super) fn body_get_shape(&self, body: Rid, shape_idx: i32) -> Rid {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body
                .get_base()
                .get_shape(&physics_data.ids, shape_idx as usize);
        }
        Rid::Invalid
    }

    pub(super) fn body_get_shape_transform(&self, body: Rid, shape_idx: i32) -> Transform {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_base().get_shape_transform(shape_idx as usize);
        }
        Transform::IDENTITY
    }

    pub(super) fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_shape_disabled(
                shape_idx as usize,
                disabled,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn body_set_shape_as_one_way_collision(
        &mut self,
        body: Rid,
        shape_idx: i32,
        enable: bool,
        margin: f32,
    ) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base()
                .set_shape_as_one_way_collision(shape_idx as usize, enable, margin);
        }
    }

    pub(super) fn body_remove_shape(&mut self, body: Rid, shape_idx: i32) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.remove_shape_idx(
                shape_idx as usize,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.shapes,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_clear_shapes(&mut self, body: Rid) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            while body.get_base().get_shape_count() > 0 {
                body.remove_shape_idx(
                    0,
                    &mut physics_data.physics_engine,
                    &mut physics_data.spaces,
                    &mut physics_data.shapes,
                    &physics_data.ids,
                );
            }
        }
    }

    pub(super) fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base().set_instance_id(id);
        }
    }

    pub(super) fn body_get_object_instance_id(&self, body: Rid) -> u64 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body) {
            return body.get_base().get_instance_id();
        }
        0
    }

    #[cfg(feature = "dim2")]
    pub(super) fn body_attach_canvas_instance_id(&mut self, body: Rid, id: u64) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base().set_canvas_instance_id(id);
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn body_get_canvas_instance_id(&self, body: Rid) -> u64 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body) {
            return body.get_base().get_canvas_instance_id();
        }
        0
    }

    #[cfg(feature = "dim2")]
    pub(super) fn body_set_continuous_collision_detection_mode(
        &mut self,
        body: Rid,
        mode: CcdMode,
    ) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_continuous_collision_detection_mode(
                mode != CcdMode::DISABLED,
                &mut physics_data.physics_engine,
            );
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn body_get_continuous_collision_detection_mode(&self, body: Rid) -> CcdMode {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
            && body.get_continuous_collision_detection_mode()
        {
            return CcdMode::CAST_RAY;
        }
        CcdMode::DISABLED
    }

    #[cfg(feature = "dim3")]
    pub fn body_set_enable_continuous_collision_detection(&mut self, body: Rid, enable: bool) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_continuous_collision_detection_mode(enable, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn body_is_continuous_collision_detection_enabled(&self, body: Rid) -> bool {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
            && body.get_continuous_collision_detection_mode()
        {
            return true;
        }
        false
    }

    pub(super) fn body_set_collision_layer(&mut self, body: Rid, layer: u32) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base()
                .set_collision_layer(layer, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_get_collision_layer(&self, body: Rid) -> u32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body) {
            return body.get_base().get_collision_layer();
        }
        0
    }

    pub(super) fn body_set_collision_mask(&mut self, body: Rid, mask: u32) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base()
                .set_collision_mask(mask, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_get_collision_mask(&self, body: Rid) -> u32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body) {
            return body.get_base().get_collision_mask();
        }
        0
    }

    pub(super) fn body_set_collision_priority(&mut self, _body: Rid, _priority: f32) {}

    pub(super) fn body_get_collision_priority(&self, _body: Rid) -> f32 {
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(super) fn body_set_user_flags(&mut self, body: Rid, flags: u32) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base().set_user_flags(flags);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn body_get_user_flags(&self, body: Rid) -> u32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body) {
            return body.get_base().get_user_flags();
        }
        0
    }

    pub(super) fn body_set_param(&mut self, body: Rid, param: BodyParameter, value: Variant) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_param(
                param,
                value,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_get_param(&self, body: Rid, param: BodyParameter) -> Variant {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_param(param);
        }
        Variant::nil()
    }

    pub(super) fn body_reset_mass_properties(&mut self, body: Rid) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.reset_mass_properties(
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_set_state(&mut self, body: Rid, state: BodyState, value: Variant) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_state(
                state,
                value,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_get_state(&self, body: Rid, state: BodyState) -> Variant {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_state(state, &physics_data.physics_engine);
        }
        Variant::nil()
    }

    pub(super) fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.apply_central_impulse(impulse, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_apply_torque_impulse(&mut self, body: Rid, impulse: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.apply_torque_impulse(impulse, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_apply_impulse(&mut self, body: Rid, impulse: Vector, position: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.apply_impulse(impulse, position, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_apply_central_force(&mut self, body: Rid, force: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.apply_central_force(force, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_apply_force(&mut self, body: Rid, force: Vector, position: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.apply_force(force, position, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_apply_torque(&mut self, body: Rid, torque: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.apply_torque(torque, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_add_constant_central_force(&mut self, body: Rid, force: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.add_constant_central_force(force, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_add_constant_force(&mut self, body: Rid, force: Vector, position: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.add_constant_force(force, position, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_add_constant_torque(&mut self, body: Rid, torque: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.add_constant_torque(torque, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_set_constant_force(&mut self, body: Rid, force: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.set_constant_force(force, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_get_constant_force(&self, body: Rid) -> Vector {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_constant_force(&physics_data.physics_engine);
        }
        Vector::default()
    }

    pub(super) fn body_set_constant_torque(&mut self, body: Rid, torque: Angle) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.force_mass_update(
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &physics_data.ids,
            );
            body.set_constant_torque(torque, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_get_constant_torque(&self, body: Rid) -> Angle {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_constant_torque(&physics_data.physics_engine);
        }
        ANGLE_ZERO
    }

    pub(super) fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            let mut v = body.get_linear_velocity(&physics_data.physics_engine);
            let axis = vector_normalized(axis_velocity);
            v -= axis * axis.dot(v);
            v += axis_velocity;
            body.set_linear_velocity(v, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn body_set_axis_lock(&mut self, body: Rid, axis: BodyAxis, lock: bool) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_axis_lock(axis, lock, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn body_is_axis_locked(&self, body: Rid, axis: BodyAxis) -> bool {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.is_axis_locked(axis);
        }
        false
    }

    pub(super) fn body_add_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.add_exception(excepted_body, &mut physics_data.physics_engine);
            body.wakeup(&mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_remove_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.remove_exception(excepted_body, &mut physics_data.physics_engine);
            body.wakeup(&mut physics_data.physics_engine);
        }
    }

    pub(super) fn body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            let exceptions = body.get_exceptions();
            let mut arr = Array::new();
            for e in exceptions {
                arr.push(*e);
            }
            return arr;
        }
        Array::new()
    }

    pub(super) fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_max_contacts_reported(
                amount,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_get_max_contacts_reported(&self, body: Rid) -> i32 {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_max_contacts_reported();
        }
        0
    }

    pub(super) fn body_set_contacts_reported_depth_threshold(
        &mut self,
        _body: Rid,
        _threshold: f32,
    ) {
    }

    pub(super) fn body_get_contacts_reported_depth_threshold(&self, _body: Rid) -> f32 {
        0.0
    }

    pub(super) fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool) {
        let physics_data = physics_data();
        let mut body_id = RapierId::default();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_omit_force_integration(enable);
            body_id = body.get_base().get_id();
        }
        if body_id != RapierId::default() {
            RapierBody::apply_area_override_to_body(
                &body_id,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
        }
    }

    pub(super) fn body_is_omitting_force_integration(&self, body: Rid) -> bool {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
        {
            return body.get_omit_force_integration();
        }
        false
    }

    pub(super) fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_state_sync_callback(callable, &mut physics_data.spaces, &physics_data.ids);
        }
    }

    pub(super) fn body_set_force_integration_callback(
        &mut self,
        body: Rid,
        callable: Callable,
        userdata: Variant,
    ) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.set_force_integration_callback(
                callable,
                userdata,
                &mut physics_data.spaces,
                &physics_data.ids,
            );
        }
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub(super) unsafe fn body_collide_shape(
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
        let physics_data = physics_data();
        let mut body_shape_rid = Rid::Invalid;
        let mut body_transform = Transform::IDENTITY;
        let mut body_shape_transform = Transform::IDENTITY;
        {
            if let Some(body) = physics_data.collision_objects.get(&body)
                && let Some(body) = body.get_body()
            {
                body_shape_rid = body
                    .get_base()
                    .get_shape(&physics_data.ids, body_shape as usize);
                body_transform = body.get_base().get_transform();
                body_shape_transform = body.get_base().get_shape_transform(body_shape as usize);
            }
        }
        unsafe {
            self.shape_collide(
                body_shape_rid,
                body_transform * body_shape_transform,
                Vector::ZERO,
                shape,
                shape_xform,
                motion,
                results,
                result_max,
                result_count,
            )
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn body_set_pickable(&mut self, body: Rid, pickable: bool) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base().set_pickable(pickable);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn body_set_ray_pickable(&mut self, body: Rid, pickable: bool) {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body) {
            body.get_mut_base().set_pickable(pickable);
        }
    }

    pub(super) fn body_get_direct_state(
        &mut self,
        body: Rid,
    ) -> Option<Gd<PhysicsDirectBodyState>> {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get_mut(&body)
            && let Some(body) = body.get_mut_body()
        {
            body.create_direct_state();
            return body.get_direct_state().cloned();
        }
        None
    }

    #[allow(clippy::too_many_arguments)]
    pub unsafe fn body_test_motion(
        &self,
        body: Rid,
        from: Transform,
        motion: Vector,
        margin: f32,
        _max_collisions: i32,
        collide_separation_ray: bool,
        recovery_as_collision: bool,
        result: *mut PhysicsServerExtensionMotionResult,
    ) -> bool {
        let physics_data = physics_data();
        if let Some(body) = physics_data.collision_objects.get(&body)
            && let Some(body) = body.get_body()
            && let Some(space) = physics_data
                .spaces
                .get(&body.get_base().get_space(&physics_data.ids))
        {
            let result: &mut PhysicsServerExtensionMotionResult = unsafe { &mut *result };
            return space.test_body_motion(
                body,
                from,
                motion,
                margin,
                collide_separation_ray,
                recovery_as_collision,
                result,
                &physics_data.physics_engine,
                &physics_data.shapes,
                &physics_data.ids,
                &physics_data.collision_objects,
            );
        }
        false
    }

    pub(super) fn joint_create(&mut self) -> Rid {
        let physics_data = physics_data();
        let rid = rid_from_int64(rid_allocate_id());
        let id = self.next_id();
        let joint = RapierEmptyJoint::new(id);
        physics_data
            .joints
            .insert(rid, RapierJoint::RapierEmptyJoint(joint));
        insert_id_rid(id, rid, &mut physics_data.ids);
        rid
    }

    pub(super) fn joint_clear(&mut self, rid: Rid) {
        let physics_data = physics_data();
        if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
            let id = self.next_id();
            let mut joint = RapierEmptyJoint::new(id);
            joint
                .get_mut_base()
                .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
            prev_joint
                .get_mut_base()
                .destroy_joint(&mut physics_data.physics_engine);
            physics_data
                .joints
                .insert(rid, RapierJoint::RapierEmptyJoint(joint));
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn joint_set_param(&mut self, joint: Rid, param: JointParam, value: f32) {
        let physics_data = physics_data();
        if let Some(joint) = physics_data.joints.get_mut(&joint)
            && param == JointParam::MAX_FORCE
        {
            joint.get_mut_base().set_max_force(value);
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn joint_get_param(&self, joint: Rid, param: JointParam) -> f32 {
        let physics_data = physics_data();
        if let Some(joint) = physics_data.joints.get(&joint)
            && param == JointParam::MAX_FORCE
        {
            return joint.get_base().get_max_force();
        }
        0.0
    }

    pub(super) fn joint_disable_collisions_between_bodies(&mut self, joint: Rid, disable: bool) {
        let physics_data = physics_data();
        if let Some(joint) = physics_data.joints.get_mut(&joint) {
            joint
                .get_mut_base()
                .disable_collisions_between_bodies(disable, &mut physics_data.physics_engine);
        }
    }

    pub(super) fn joint_is_disabled_collisions_between_bodies(&self, joint: Rid) -> bool {
        let physics_data = physics_data();
        if let Some(joint) = physics_data.joints.get(&joint) {
            return joint.get_base().is_disabled_collisions_between_bodies();
        }
        true
    }

    #[cfg(feature = "dim3")]
    pub(super) fn joint_set_solver_priority(&mut self, _joint: Rid, _priority: i32) {}

    #[cfg(feature = "dim3")]
    pub(super) fn joint_get_solver_priority(&self, _joint: Rid) -> i32 {
        1
    }

    #[cfg(feature = "dim3")]
    pub(super) fn joint_make_pin(
        &mut self,
        rid: Rid,
        body_a: Rid,
        local_a: Vector3,
        body_b: Rid,
        local_b: Vector3,
    ) {
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierSphericalJoint3D(RapierSphericalJoint3D::new(
                id,
                rid,
                local_a,
                local_b,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim3")]
    pub(super) fn pin_joint_set_param(&mut self, joint: Rid, param: PinJointParam, value: f32) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSphericalJoint3D(joint)) =
            physics_data.joints.get_mut(&joint)
        {
            joint.set_param(param, value, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn pin_joint_get_param(&self, joint: Rid, param: PinJointParam) -> f32 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSphericalJoint3D(joint)) = physics_data.joints.get(&joint) {
            return joint.get_param(param);
        }
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(super) fn pin_joint_set_local_a(&mut self, joint: Rid, local_a: Vector3) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSphericalJoint3D(joint)) =
            physics_data.joints.get_mut(&joint)
        {
            joint.set_anchor_a(local_a, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn pin_joint_get_local_a(&self, joint: Rid) -> Vector3 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSphericalJoint3D(joint)) = physics_data.joints.get(&joint) {
            return joint.get_anchor_a();
        }
        Vector3::ZERO
    }

    #[cfg(feature = "dim3")]
    pub(super) fn pin_joint_set_local_b(&mut self, joint: Rid, local_b: Vector3) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSphericalJoint3D(joint)) =
            physics_data.joints.get_mut(&joint)
        {
            joint.set_anchor_b(local_b, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn pin_joint_get_local_b(&self, joint: Rid) -> Vector3 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSphericalJoint3D(joint)) = physics_data.joints.get(&joint) {
            return joint.get_anchor_b();
        }
        Vector3::ZERO
    }

    #[cfg(feature = "dim3")]
    pub(super) fn joint_make_hinge(
        &mut self,
        rid: Rid,
        body_a: Rid,
        hinge_a: Transform3D,
        body_b: Rid,
        hinge_b: Transform3D,
    ) {
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierRevoluteJoint(RapierRevoluteJoint::new(
                id,
                rid,
                hinge_a.origin,
                hinge_b.origin,
                hinge_a.basis,
                hinge_b.basis,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub(super) fn joint_make_hinge_simple(
        &mut self,
        rid: Rid,
        body_a: Rid,
        pivot_a: Vector3,
        axis_a: Vector3,
        body_b: Rid,
        pivot_b: Vector3,
        axis_b: Vector3,
    ) {
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            // Create basis from axis vectors
            // The hinge axis should be the X-axis in the local frame
            // Construct a basis where X-axis is aligned with the given axis
            let basis_a = if axis_a.length_squared() > 0.0 {
                let x_axis = axis_a.normalized();
                // Choose an arbitrary perpendicular vector for Y
                let y_axis = if x_axis.abs().dot(Vector3::UP) < 0.99 {
                    x_axis.cross(Vector3::UP).normalized()
                } else {
                    x_axis.cross(Vector3::RIGHT).normalized()
                };
                let z_axis = x_axis.cross(y_axis).normalized();
                godot::prelude::Basis::from_cols(x_axis, y_axis, z_axis)
            } else {
                godot::prelude::Basis::IDENTITY
            };
            let basis_b = if axis_b.length_squared() > 0.0 {
                let x_axis = axis_b.normalized();
                let y_axis = if x_axis.abs().dot(Vector3::UP) < 0.99 {
                    x_axis.cross(Vector3::UP).normalized()
                } else {
                    x_axis.cross(Vector3::RIGHT).normalized()
                };
                let z_axis = x_axis.cross(y_axis).normalized();
                godot::prelude::Basis::from_cols(x_axis, y_axis, z_axis)
            } else {
                godot::prelude::Basis::IDENTITY
            };
            let id = self.next_id();
            joint = RapierJoint::RapierRevoluteJoint(RapierRevoluteJoint::new(
                id,
                rid,
                pivot_a,
                pivot_b,
                basis_a,
                basis_b,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim3")]
    pub(super) fn hinge_joint_set_param(&mut self, joint: Rid, param: HingeJointParam, value: f32) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get_mut(&joint) {
            joint.set_param(param, value, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn hinge_joint_get_param(&self, joint: Rid, param: HingeJointParam) -> f32 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get(&joint) {
            return joint.get_param(param);
        }
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(super) fn hinge_joint_set_flag(&mut self, joint: Rid, flag: HingeJointFlag, enabled: bool) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get_mut(&joint) {
            joint.set_flag(flag, enabled, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn hinge_joint_get_flag(&self, joint: Rid, flag: HingeJointFlag) -> bool {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get(&joint) {
            return joint.get_flag(flag);
        }
        false
    }

    #[cfg(feature = "dim3")]
    pub(super) fn joint_make_slider(
        &mut self,
        rid: Rid,
        body_a: Rid,
        local_ref_a: Transform3D,
        body_b: Rid,
        local_ref_b: Transform3D,
    ) {
        use crate::joints::rapier_slider_joint_3d::RapierSliderJoint3D;
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierSliderJoint3D(RapierSliderJoint3D::new(
                id,
                rid,
                local_ref_a.origin,
                local_ref_b.origin,
                local_ref_a.basis,
                local_ref_b.basis,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim3")]
    pub(super) fn slider_joint_set_param(
        &mut self,
        joint: Rid,
        param: physics_server_3d::SliderJointParam,
        value: f32,
    ) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSliderJoint3D(joint)) = physics_data.joints.get_mut(&joint) {
            joint.set_param(param, value, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn slider_joint_get_param(
        &self,
        joint: Rid,
        param: physics_server_3d::SliderJointParam,
    ) -> f32 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierSliderJoint3D(joint)) = physics_data.joints.get(&joint) {
            return joint.get_param(param);
        }
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(super) fn joint_make_cone_twist(
        &mut self,
        rid: Rid,
        body_a: Rid,
        local_ref_a: Transform3D,
        body_b: Rid,
        local_ref_b: Transform3D,
    ) {
        use crate::joints::rapier_cone_twist_joint_3d::RapierConeTwistJoint3D;
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierConeTwistJoint3D(RapierConeTwistJoint3D::new(
                id,
                rid,
                local_ref_a.origin,
                local_ref_b.origin,
                local_ref_a.basis,
                local_ref_b.basis,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim3")]
    pub(super) fn cone_twist_joint_set_param(
        &mut self,
        joint: Rid,
        param: physics_server_3d::ConeTwistJointParam,
        value: f32,
    ) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierConeTwistJoint3D(joint)) =
            physics_data.joints.get_mut(&joint)
        {
            joint.set_param(param, value, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn cone_twist_joint_get_param(
        &self,
        joint: Rid,
        param: physics_server_3d::ConeTwistJointParam,
    ) -> f32 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierConeTwistJoint3D(joint)) = physics_data.joints.get(&joint) {
            return joint.get_param(param);
        }
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(super) fn joint_make_generic_6dof(
        &mut self,
        rid: Rid,
        body_a: Rid,
        local_ref_a: Transform3D,
        body_b: Rid,
        local_ref_b: Transform3D,
    ) {
        use crate::joints::rapier_generic_6dof_joint_3d::RapierGeneric6DOFJoint3D;
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierGeneric6DOFJoint3D(RapierGeneric6DOFJoint3D::new(
                id,
                rid,
                local_ref_a.origin,
                local_ref_b.origin,
                local_ref_a.basis,
                local_ref_b.basis,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim3")]
    pub(super) fn generic_6dof_joint_set_param(
        &mut self,
        joint: Rid,
        axis: Vector3Axis,
        param: physics_server_3d::G6dofJointAxisParam,
        value: f32,
    ) {
        use rapier::prelude::JointAxis;
        let physics_data = physics_data();
        if let Some(joint_data) = physics_data.joints.get(&joint) {
            // Determine if this is a linear or angular parameter
            let is_angular = matches!(
                param,
                physics_server_3d::G6dofJointAxisParam::ANGULAR_LOWER_LIMIT
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_UPPER_LIMIT
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_LIMIT_SOFTNESS
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_RESTITUTION
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_DAMPING
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_MOTOR_TARGET_VELOCITY
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_MOTOR_FORCE_LIMIT
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_STIFFNESS
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_DAMPING
                    | physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_EQUILIBRIUM_POINT
            );
            let rapier_axis = if is_angular {
                match axis {
                    Vector3Axis::X => JointAxis::AngX,
                    Vector3Axis::Y => JointAxis::AngY,
                    Vector3Axis::Z => JointAxis::AngZ,
                }
            } else {
                match axis {
                    Vector3Axis::X => JointAxis::LinX,
                    Vector3Axis::Y => JointAxis::LinY,
                    Vector3Axis::Z => JointAxis::LinZ,
                }
            };
            physics_data
                .physics_engine
                .joint_change_generic_6dof_axis_param(
                    joint_data.get_base().get_space_id(),
                    joint_data.get_base().get_handle(),
                    rapier_axis,
                    param,
                    value,
                );
        }
        // Store the parameter in the joint struct (separate borrow after immutable borrow ends)
        if let Some(RapierJoint::RapierGeneric6DOFJoint3D(joint_6dof)) =
            physics_data.joints.get_mut(&joint)
        {
            joint_6dof.set_param(axis, param, value);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn generic_6dof_joint_get_param(
        &self,
        joint: Rid,
        axis: Vector3Axis,
        param: physics_server_3d::G6dofJointAxisParam,
    ) -> f32 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierGeneric6DOFJoint3D(joint_6dof)) =
            physics_data.joints.get(&joint)
        {
            return joint_6dof.get_param(axis, param);
        }
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(super) fn generic_6dof_joint_set_flag(
        &mut self,
        joint: Rid,
        axis: Vector3Axis,
        flag: physics_server_3d::G6dofJointAxisFlag,
        enable: bool,
    ) {
        use rapier::prelude::JointAxis;
        let physics_data = physics_data();
        if let Some(joint_data) = physics_data.joints.get(&joint) {
            // Determine if this is a linear or angular flag
            let is_angular = matches!(
                flag,
                physics_server_3d::G6dofJointAxisFlag::ENABLE_ANGULAR_LIMIT
                    | physics_server_3d::G6dofJointAxisFlag::ENABLE_ANGULAR_SPRING
            );
            let rapier_axis = if is_angular {
                match axis {
                    Vector3Axis::X => JointAxis::AngX,
                    Vector3Axis::Y => JointAxis::AngY,
                    Vector3Axis::Z => JointAxis::AngZ,
                }
            } else {
                match axis {
                    Vector3Axis::X => JointAxis::LinX,
                    Vector3Axis::Y => JointAxis::LinY,
                    Vector3Axis::Z => JointAxis::LinZ,
                }
            };
            physics_data
                .physics_engine
                .joint_change_generic_6dof_axis_flag(
                    joint_data.get_base().get_space_id(),
                    joint_data.get_base().get_handle(),
                    rapier_axis,
                    flag,
                    enable,
                );
        }
        // Store the flag in the joint struct (separate borrow after immutable borrow ends)
        if let Some(RapierJoint::RapierGeneric6DOFJoint3D(joint_6dof)) =
            physics_data.joints.get_mut(&joint)
        {
            joint_6dof.set_flag(axis, flag, enable);
        }
    }

    #[cfg(feature = "dim3")]
    pub(super) fn generic_6dof_joint_get_flag(
        &self,
        joint: Rid,
        axis: Vector3Axis,
        flag: physics_server_3d::G6dofJointAxisFlag,
    ) -> bool {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierGeneric6DOFJoint3D(joint_6dof)) =
            physics_data.joints.get(&joint)
        {
            return joint_6dof.get_flag(axis, flag);
        }
        false
    }

    #[cfg(feature = "dim2")]
    pub(super) fn joint_make_pin(&mut self, rid: Rid, anchor: Vector, body_a: Rid, body_b: Rid) {
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierRevoluteJoint(RapierRevoluteJoint::new(
                id,
                rid,
                anchor,
                anchor,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim2")]
    pub(super) fn joint_make_groove(
        &mut self,
        rid: Rid,
        a_groove1: Vector,
        a_groove2: Vector,
        b_anchor: Vector,
        body_a: Rid,
        body_b: Rid,
    ) {
        let physics_data: &mut super::rapier_physics_singleton::PhysicsData = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierGrooveJoint2D(RapierGrooveJoint2D::new(
                id,
                rid,
                a_groove1,
                a_groove2,
                b_anchor,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim2")]
    pub(super) fn joint_make_damped_spring(
        &mut self,
        rid: Rid,
        anchor_a: Vector,
        anchor_b: Vector,
        body_a: Rid,
        body_b: Rid,
    ) {
        let physics_data = physics_data();
        let mut joint: RapierJoint;
        let joint_type = if let Some(prev_joint) = physics_data.joints.get(&rid) {
            prev_joint.get_base().get_joint_type()
        } else {
            RapierJointType::Impulse
        };
        if let Some(body_a) = physics_data.collision_objects.get(&body_a)
            && let Some(body_b) = physics_data.collision_objects.get(&body_b)
        {
            let id = self.next_id();
            joint = RapierJoint::RapierDampedSpringJoint2D(RapierDampedSpringJoint2D::new(
                id,
                rid,
                anchor_a,
                anchor_b,
                body_a,
                body_b,
                &mut physics_data.physics_engine,
                joint_type,
            ));
            if let Some(mut prev_joint) = physics_data.joints.remove(&rid) {
                joint
                    .get_mut_base()
                    .copy_settings_from(prev_joint.get_base(), &mut physics_data.physics_engine);
                prev_joint
                    .get_mut_base()
                    .destroy_joint(&mut physics_data.physics_engine);
            }
        } else {
            let id = self.next_id();
            joint = RapierJoint::RapierEmptyJoint(RapierEmptyJoint::new(id));
        }
        physics_data.joints.insert(rid, joint);
    }

    #[cfg(feature = "dim2")]
    pub(super) fn pin_joint_set_flag(&mut self, joint: Rid, flag: PinJointFlag, enabled: bool) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get_mut(&joint) {
            joint.set_flag(flag, enabled, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn pin_joint_get_flag(&self, joint: Rid, flag: PinJointFlag) -> bool {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get(&joint) {
            return joint.get_flag(flag);
        }
        false
    }

    #[cfg(feature = "dim2")]
    pub(super) fn pin_joint_set_param(&mut self, joint: Rid, param: PinJointParam, value: f32) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get_mut(&joint) {
            joint.set_param(param, value, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn pin_joint_get_param(&self, joint: Rid, param: PinJointParam) -> f32 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierRevoluteJoint(joint)) = physics_data.joints.get(&joint) {
            return joint.get_param(param);
        }
        0.0
    }

    #[cfg(feature = "dim2")]
    pub(super) fn damped_spring_joint_set_param(
        &mut self,
        joint: Rid,
        param: DampedSpringParam,
        value: f32,
    ) {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierDampedSpringJoint2D(joint)) =
            physics_data.joints.get_mut(&joint)
        {
            joint.set_param(param, value, &mut physics_data.physics_engine);
        }
    }

    #[cfg(feature = "dim2")]
    pub(super) fn damped_spring_joint_get_param(
        &self,
        joint: Rid,
        param: DampedSpringParam,
    ) -> f32 {
        let physics_data = physics_data();
        if let Some(RapierJoint::RapierDampedSpringJoint2D(joint)) = physics_data.joints.get(&joint)
        {
            return joint.get_param(param);
        }
        0.0
    }

    pub(super) fn joint_get_type(&self, joint: Rid) -> JointType {
        let physics_data = physics_data();
        if let Some(joint) = physics_data.joints.get(&joint) {
            return joint.get_type();
        }
        JointType::MAX
    }

    pub(super) fn free_rid(&mut self, rid: Rid) {
        let physics_data = physics_data();
        let mut space_to_reset = Rid::Invalid;
        if let Some(mut shape) = physics_data.shapes.remove(&rid) {
            for (owner, _) in shape.get_base().get_owners() {
                if let Some(body) = physics_data
                    .collision_objects
                    .get_mut(&get_id_rid(*owner, &physics_data.ids))
                {
                    space_to_reset = body.get_base().get_space(&physics_data.ids);
                    body.remove_shape_rid(
                        shape.get_base().get_rid(),
                        &mut physics_data.physics_engine,
                        &mut physics_data.spaces,
                        &mut physics_data.shapes,
                        &physics_data.ids,
                    );
                }
            }
            shape
                .get_mut_base()
                .destroy_shape(&mut physics_data.physics_engine);
            remove_id_rid(shape.get_base().get_id(), &mut physics_data.ids);
        } else if let Some(mut body) = physics_data.collision_objects.remove(&rid) {
            space_to_reset = body.get_base().get_space(&physics_data.ids);
            body.set_space(
                Rid::Invalid,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.ids,
            );
            remove_id_rid(body.get_base().get_id(), &mut physics_data.ids);
        } else if let Some(mut space) = physics_data.spaces.remove(&rid) {
            let space_handle = space.get_state().get_id();
            remove_id_rid(space.get_state().get_id(), &mut physics_data.ids);
            space
                .get_mut_state()
                .destroy(&mut physics_data.physics_engine);
            physics_data.active_spaces.remove(&space_handle);
        } else if let Some(mut joint) = physics_data.joints.remove(&rid) {
            space_to_reset = joint.get_base().get_space(&physics_data.ids);
            joint
                .get_mut_base()
                .destroy_joint(&mut physics_data.physics_engine);
            remove_id_rid(joint.get_base().get_id(), &mut physics_data.ids);
        } else if let Some(mut fluid) = physics_data.fluids.remove(&rid) {
            fluid.destroy_fluid(&mut physics_data.physics_engine);
            remove_id_rid(fluid.get_id(), &mut physics_data.ids);
        }
        if let Some(space) = physics_data.spaces.get_mut(&space_to_reset) {
            space.get_mut_state().reset_space_if_empty(
                &mut physics_data.physics_engine,
                &RapierSpace::get_world_settings(),
            );
        }
        // If there are no more objects, reset the ids. Ensures there will be determinism.
        if physics_data.collision_objects.is_empty()
            && physics_data.spaces.is_empty()
            && physics_data.joints.is_empty()
            && physics_data.fluids.is_empty()
            && physics_data.shapes.is_empty()
            && physics_data.ids.is_empty()
        {
            self.id = RapierId::default();
        }
    }

    pub(super) fn set_active(&mut self, active: bool) {
        self.active = active;
    }

    pub(super) fn init_ext(&mut self) {
        self.active = true;
        self.stepped = false;
        self.flushing_queries = false;
        self.doing_sync = false;
        self.active_objects = 0;
        self.id = RapierId::default();
    }

    pub(super) fn step(&mut self, step: f32) {
        let physics_data = physics_data();
        if !self.active {
            return;
        }
        self.stepped = true;
        self.active_objects = 0;
        let active_spaces = physics_data.active_spaces.clone();
        for space_rid in active_spaces.values() {
            self.space_step(space_rid, step);
            if let Some(space) = physics_data.spaces.get(space_rid) {
                self.active_objects += space.get_state().get_active_objects();
            }
        }
    }

    pub(super) fn sync(&mut self) {
        self.doing_sync = true;
    }

    pub(super) fn end_sync(&mut self) {
        self.doing_sync = false;
    }

    pub(super) fn finish(&mut self) {}

    pub(super) fn is_flushing_queries(&self) -> bool {
        self.flushing_queries
    }

    pub(super) fn get_process_info(&mut self, process_info: ProcessInfo) -> i32 {
        match process_info {
            ProcessInfo::ACTIVE_OBJECTS => self.active_objects,
            _ => 0,
        }
    }

    pub(super) fn space_step(&mut self, space_rid: &Rid, step: f32) {
        let physics_data = physics_data();
        if !self.active {
            return;
        }
        let settings = SimulationSettings {
            dt: step,
            length_unit: self.length_unit,
            max_ccd_substeps: self.max_ccd_substeps,
            num_internal_pgs_iterations: self.num_internal_pgs_iterations,
            num_solver_iterations: self.num_solver_iterations,
            pixel_gravity: vector_to_rapier(Vector::ZERO),
            pixel_liquid_gravity: vector_to_rapier(Vector::ZERO),
            normalized_allowed_linear_error: self.normalized_allowed_linear_error,
            normalized_max_corrective_velocity: self.normalized_max_corrective_velocity,
            normalized_prediction_distance: self.normalized_prediction_distance,
            predictive_contact_allowance_threshold: self.predictive_contact_allowance_threshold,
            num_internal_stabilization_iterations: self.num_internal_stabilization_iterations,
            contact_damping_ratio: self.contact_damping_ratio,
            contact_natural_frequency: self.contact_natural_frequency,
        };
        RapierSpace::step(step, space_rid, physics_data, settings);
    }

    pub(super) fn space_flush_queries(space: &Rid) {
        let physics_data = physics_data();
        let mut state_query_list = None;
        let mut force_integrate_query_list = None;
        let mut monitor_query_list = None;
        if let Some(space) = physics_data.spaces.get_mut(space) {
            state_query_list = Some(space.get_state().get_state_query_list());
            force_integrate_query_list = Some(space.get_state().get_force_integrate_query_list());
            monitor_query_list = Some(space.get_state().get_monitor_query_list());
        }
        if let Some(state_query_list) = state_query_list
            && let Some(force_integrate_query_list) = force_integrate_query_list
            && let Some(monitor_query_list) = monitor_query_list
        {
            RapierSpace::call_queries(
                state_query_list,
                force_integrate_query_list,
                monitor_query_list,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
        }
        if let Some(space) = physics_data.spaces.get_mut(space) {
            space.update_after_queries(&mut physics_data.collision_objects, &physics_data.ids);
        }
    }

    pub(super) fn get_id(&self, rid: Rid) -> u64 {
        let physics_data = physics_data();
        if let Some(shape) = physics_data.shapes.get(&rid) {
            return shape.get_base().get_id();
        } else if let Some(body) = physics_data.collision_objects.get(&rid) {
            return body.get_base().get_id();
        } else if let Some(space) = physics_data.spaces.get(&rid) {
            return space.get_state().get_id();
        } else if let Some(joint) = physics_data.joints.get(&rid) {
            return joint.get_base().get_id();
        } else if let Some(fluid) = physics_data.fluids.get(&rid) {
            return fluid.get_id();
        }
        0
    }
}
