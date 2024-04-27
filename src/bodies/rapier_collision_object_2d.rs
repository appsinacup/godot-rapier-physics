use godot::engine::native::ObjectId;
use godot::engine::physics_server_2d;
use godot::prelude::*;

use crate::rapier2d::handle::{invalid_handle, Handle};

pub struct Shape {
    xform: Transform2D,
    shape: Option<Rid>,
    disabled: bool,
    one_way_collision: bool,
    one_way_collision_margin: f32,
    collider_handle: Handle,
}

pub struct RapierCollisionObject2D {
    type_: Type,
    rid: Rid,
    instance_id: ObjectId,
    canvas_instance_id: ObjectId,
    pickable: bool,
    shapes: Vec<Shape>,
    space: Option<Rid>,
    transform: Transform2D,
    inv_transform: Transform2D,
    collision_mask: u32,
    collision_layer: u32,
    collision_priority: f32,
    mode: physics_server_2d::BodyMode,
    body_handle: Handle,
    area_detection_counter: u32,
}

pub enum Type {
    Area,
    Body,
}

impl RapierCollisionObject2D {
    pub fn new(type_: Type) -> Self {
        Self {
            type_,
            rid: Rid::Invalid,
            instance_id: ObjectId { id: 0 },
            canvas_instance_id: ObjectId { id: 0 },
            pickable: true,
            shapes: Vec::new(),
            space: None,
            transform: Transform2D::IDENTITY,
            inv_transform: Transform2D::IDENTITY,
            collision_mask: 1,
            collision_layer: 1,
            collision_priority: 1.0,
            mode: physics_server_2d::BodyMode::STATIC,
            body_handle: invalid_handle(),
            area_detection_counter: 0,
        }
    }
}
