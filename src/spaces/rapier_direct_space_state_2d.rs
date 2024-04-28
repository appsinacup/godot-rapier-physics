use crate::spaces::rapier_space_2d::RapierSpace2D;
use godot::{
    engine::{IPhysicsDirectSpaceState2DExtension, PhysicsDirectSpaceState2DExtension},
    prelude::*,
};
use std::{cell::RefCell, rc::Rc};

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState2DExtension)]
pub struct RapierDirectSpaceState2D {
    space: Option<Rc<RefCell<RapierSpace2D>>>,

    base: Base<PhysicsDirectSpaceState2DExtension>,
}

#[godot_api]
impl IPhysicsDirectSpaceState2DExtension for RapierDirectSpaceState2D {
    fn init(base: Base<PhysicsDirectSpaceState2DExtension>) -> Self {
        Self { space: None, base }
    }

    unsafe fn intersect_ray(
        &mut self,
        from: Vector2,
        to: Vector2,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        result: *mut godot::engine::native::PhysicsServer2DExtensionRayResult,
    ) -> bool {
        false
    }

    unsafe fn intersect_point(
        &mut self,
        position: Vector2,
        canvas_instance_id: u64,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut godot::engine::native::PhysicsServer2DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        0
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
        result: *mut godot::engine::native::PhysicsServer2DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        0
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
        unimplemented!()
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
        false
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
        rest_info: *mut godot::engine::native::PhysicsServer2DExtensionShapeRestInfo,
    ) -> bool {
        false
    }
}
