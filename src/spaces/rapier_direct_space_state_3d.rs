use godot::classes::*;
use godot::meta::RawPtr;
use godot::prelude::*;

use super::rapier_direct_space_state_impl::RapierDirectSpaceStateImpl;
use crate::servers::rapier_physics_singleton::physics_data;
use crate::types::*;
#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState3DExtension,tool)]
/// The physics direct space state singleton implemented for Rapier Physics.
/// For methods exposed see [PhysicsDirectSpaceState3D].
pub struct RapierDirectSpaceState3D {
    inner: RapierDirectSpaceStateImpl,
    space: Rid,
    base: Base<PhysicsDirectSpaceState3DExtension>,
}
impl RapierDirectSpaceState3D {
    pub fn set_space(&mut self, space: Rid) {
        self.space = space;
        self.inner.space = space;
    }
}
#[godot_api]
impl IPhysicsDirectSpaceState3DExtension for RapierDirectSpaceState3D {
    fn init(base: Base<PhysicsDirectSpaceState3DExtension>) -> Self {
        Self {
            inner: RapierDirectSpaceStateImpl::default(),
            space: Rid::Invalid,
            base,
        }
    }

    unsafe fn intersect_ray_rawptr(
        &mut self,
        from: Vector,
        to: Vector,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        _hit_back_faces: bool,
        _pick_ray: bool,
        result: RawPtr<*mut PhysicsServerExtensionRayResult>,
    ) -> bool {
        let physics_data = physics_data();
        unsafe {
            self.inner.intersect_ray(
                from,
                to,
                collision_mask,
                collide_with_bodies,
                collide_with_areas,
                hit_from_inside,
                result,
                physics_data,
            )
        }
    }

    unsafe fn intersect_point_rawptr(
        &mut self,
        position: Vector,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: RawPtr<*mut PhysicsServerExtensionShapeResult>,
        max_results: i32,
    ) -> i32 {
        let physics_data = physics_data();
        unsafe {
            self.inner.intersect_point(
                position,
                0,
                collision_mask,
                collide_with_bodies,
                collide_with_areas,
                results,
                max_results,
                physics_data,
            )
        }
    }

    unsafe fn intersect_shape_rawptr(
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
    ) -> i32 {
        let physics_data = physics_data();
        unsafe {
            self.inner.intersect_shape(
                shape_rid,
                transform,
                motion,
                margin,
                collision_mask,
                collide_with_bodies,
                collide_with_areas,
                results,
                max_results,
                physics_data,
            )
        }
    }

    unsafe fn cast_motion_rawptr(
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
        _info: RawPtr<*mut PhysicsServerExtensionShapeRestInfo>,
    ) -> bool {
        let physics_data = physics_data();
        unsafe {
            self.inner.cast_motion(
                shape_rid,
                transform,
                motion,
                margin,
                collision_mask,
                collide_with_bodies,
                collide_with_areas,
                closest_safe,
                closest_unsafe,
                physics_data,
            )
        }
    }

    unsafe fn collide_shape_rawptr(
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
    ) -> bool {
        let physics_data = physics_data();
        unsafe {
            self.inner.collide_shape(
                shape_rid,
                transform,
                motion,
                margin,
                collision_mask,
                collide_with_bodies,
                collide_with_areas,
                results,
                max_results,
                result_count,
                physics_data,
            )
        }
    }

    unsafe fn rest_info_rawptr(
        &mut self,
        shape_rid: Rid,
        transform: Transform,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: RawPtr<*mut PhysicsServerExtensionShapeRestInfo>,
    ) -> bool {
        let physics_data = physics_data();
        unsafe {
            self.inner.rest_info(
                shape_rid,
                transform,
                motion,
                margin,
                collision_mask,
                collide_with_bodies,
                collide_with_areas,
                rest_info,
                physics_data,
            )
        }
    }

    fn get_closest_point_to_object_volume(&self, _object: Rid, _point: Vector3) -> Vector3 {
        godot_print!("Not implemented");
        Vector::ZERO
    }
}
