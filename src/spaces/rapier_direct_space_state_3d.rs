use std::ops::Deref;

use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::*;
use crate::spaces::rapier_space::RapierSpace;
use crate::Vector;
use godot::classes::native::*;
use godot::classes::*;
use godot::prelude::*;
use rapier::math::Real;

use super::{
    rapier_direct_space_state_impl::RapierDirectSpaceStateImpl,
    rapier_space_body_helper::is_handle_excluded_callback,
};

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState3DExtension,tool)]
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
        self.inner.intersect_ray(
            from,
            to,
            collision_mask,
            collide_with_bodies,
            collide_with_areas,
            hit_from_inside,
            result,
        )
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
        self.inner.intersect_point(
            position,
            0,
            collision_mask,
            collide_with_bodies,
            collide_with_areas,
            results,
            max_results,
        )
    }

    unsafe fn intersect_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::engine::native::PhysicsServer3DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
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
        )
    }

    unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: *mut f64,
        closest_unsafe: *mut f64,
        _info: *mut PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
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
        )
    }

    unsafe fn collide_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut std::ffi::c_void,
        max_results: i32,
        result_count: *mut i32,
    ) -> bool {
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
        )
    }

    unsafe fn rest_info(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: *mut godot::engine::native::PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
        self.inner.rest_info(
            shape_rid,
            transform,
            motion,
            margin,
            collision_mask,
            collide_with_bodies,
            collide_with_areas,
            rest_info,
        )
    }
}
