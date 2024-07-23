use godot::classes::native::*;
use godot::classes::*;
use godot::prelude::*;

use super::rapier_direct_space_state_impl::RapierDirectSpaceStateImpl;
use crate::servers::rapier_physics_singleton::physics_data;
#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState2DExtension,tool)]
pub struct RapierDirectSpaceState2D {
    inner: RapierDirectSpaceStateImpl,
    space: Rid,
    base: Base<PhysicsDirectSpaceState2DExtension>,
}
impl RapierDirectSpaceState2D {
    pub fn set_space(&mut self, space: Rid) {
        self.space = space;
        self.inner.space = space;
    }
}
#[godot_api]
impl RapierDirectSpaceState2D {
    #[cfg(feature = "serde-serialize")]
    #[func]
    pub fn export_json(&mut self) -> String {
        let physics_data = physics_data();
        let Some(space) = physics_data.spaces.get(&self.space) else {
            return "{}".to_string();
        };
        space.export_json(&mut physics_data.physics_engine)
    }
}
#[godot_api]
impl IPhysicsDirectSpaceState2DExtension for RapierDirectSpaceState2D {
    #[no_mangle]
    fn init(base: Base<PhysicsDirectSpaceState2DExtension>) -> Self {
        Self {
            inner: RapierDirectSpaceStateImpl::default(),
            space: Rid::Invalid,
            base,
        }
    }

    unsafe fn intersect_ray(
        &mut self,
        from: Vector2,
        to: Vector2,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        result: *mut PhysicsServer2DExtensionRayResult,
    ) -> bool {
        let physics_data = physics_data();
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

    unsafe fn intersect_point(
        &mut self,
        position: Vector2,
        canvas_instance_id: u64,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::classes::native::PhysicsServer2DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        let physics_data = physics_data();
        self.inner.intersect_point(
            position,
            canvas_instance_id,
            collision_mask,
            collide_with_bodies,
            collide_with_areas,
            results,
            max_results,
            physics_data,
        )
    }

    unsafe fn intersect_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::classes::native::PhysicsServer2DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        let physics_data = physics_data();
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

    unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: *mut f64,
        closest_unsafe: *mut f64,
    ) -> bool {
        let physics_data = physics_data();
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

    unsafe fn collide_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut std::ffi::c_void,
        max_results: i32,
        result_count: *mut i32,
    ) -> bool {
        let physics_data = physics_data();
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

    unsafe fn rest_info(
        &mut self,
        shape_rid: Rid,
        transform: Transform2D,
        motion: Vector2,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: *mut godot::classes::native::PhysicsServer2DExtensionShapeRestInfo,
    ) -> bool {
        let physics_data = physics_data();
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
