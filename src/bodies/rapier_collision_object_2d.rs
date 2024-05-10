use std::any::Any;

use crate::{
    rapier2d::{
        body::{body_get_angle, body_get_position, body_set_transform},
        collider::{collider_create_sensor, collider_create_solid, collider_destroy, collider_set_transform, Material},
        handle::{invalid_handle, is_handle_valid, Handle},
        shape::ShapeInfo,
        user_data::UserData,
        vector::Vector,
    }, servers::rapier_physics_singleton_2d::{shapes_singleton, spaces_singleton}, shapes::rapier_shape_2d::IRapierShape2D
};
use godot::{
    builtin::{real, Rid, Transform2D, Vector2},
    engine::physics_server_2d,
};

use super::{rapier_area_2d::RapierArea2D, rapier_body_2d::RapierBody2D};

pub trait IRapierCollisionObject2D: Any {
    fn get_base(&self) -> &RapierCollisionObject2D;
    fn get_mut_base(&mut self) -> &mut RapierCollisionObject2D;
    fn get_body(&self) -> Option<&RapierBody2D>;
    fn get_area(&self) -> Option<&RapierArea2D>;
    fn get_mut_body(&mut self) -> Option<&mut RapierBody2D>;
    fn get_mut_area(&mut self) -> Option<&mut RapierArea2D>;
    fn set_space(&mut self, space: Rid);
    fn add_shape(
        &mut self,
        p_shape: Rid,
        p_transform: Transform2D,
        p_disabled: bool,
    );
    fn set_shape(
        &mut self,
        shape_idx: i32,
        p_shape: Rid,
    );
    fn set_shape_transform(&mut self, shape_idx: i32, transform: Transform2D);
    fn set_shape_disabled(&mut self, shape_idx: i32, disabled: bool);
    fn remove_shape(&mut self, shape_idx: i32);
    fn create_shape(&mut self, shape: &mut CollisionObjectShape, p_shape_index: usize, mat: Material);

    fn _init_material(&self, mat: &mut Material);
    fn _shapes_changed(&self);

    fn _shape_changed(&self, p_shape: Rid);
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CollisionObjectType {
    Area,
    Body,
}

pub struct CollisionObjectShape {
    pub xform: Transform2D,
    pub shape: Rid,
    pub disabled: bool,
    pub one_way_collision: bool,
    pub one_way_collision_margin: real,
    pub collider_handle: Handle,
}

impl Default for CollisionObjectShape {
    fn default() -> Self {
        Self {
            xform: Transform2D::default(),
            shape: Rid::Invalid,
            disabled: false,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: invalid_handle(),
        }
    }
}

pub struct RapierCollisionObject2D {
    collision_object_type: CollisionObjectType,
    rid: Rid,
    instance_id: u64,
    canvas_instance_id: u64,
    pickable: bool,
    shapes: Vec<CollisionObjectShape>,
    space: Rid,
    transform: Transform2D,
    inv_transform: Transform2D,
    collision_mask: u32,
    collision_layer: u32,
    collision_priority: real,
    pub mode: physics_server_2d::BodyMode,
    body_handle: Handle,
    area_detection_counter: u32,
}

impl RapierCollisionObject2D {
    pub fn new(rid: Rid, collision_object_type: CollisionObjectType) -> Self {
        Self {
            collision_object_type: collision_object_type,
            rid: rid,
            instance_id: 0,
            canvas_instance_id: 0,
            pickable: true,
            shapes: Vec::new(),
            space: Rid::Invalid,
            transform: Transform2D::default(),
            inv_transform: Transform2D::default(),
            collision_mask: 1,
            collision_layer: 1,
            collision_priority: 1.0,
            mode: physics_server_2d::BodyMode::RIGID,
            body_handle: invalid_handle(),
            area_detection_counter: 0,
        }
    }

    fn _create_shape(&mut self, shape: &mut CollisionObjectShape, p_shape_index: usize, mat: Material) {
        let mut space_handle = invalid_handle();
        {
            let lock = spaces_singleton().lock().unwrap();
            if let Some(space) = lock.spaces.get(&self.space) {
                space_handle = space.get_handle();
            }
        }
        assert!(is_handle_valid(space_handle));
        assert!(!is_handle_valid(shape.collider_handle));
        let mut lock = shapes_singleton().lock().unwrap();
        if let Some(shape_object) = lock.shapes.get_mut(&shape.shape) {
            let shape_handle = shape_object.get_rapier_shape();
            assert!(is_handle_valid(shape_handle));

            let mut user_data = UserData::default();
            self.set_collider_user_data(&mut user_data, p_shape_index);

            match self.collision_object_type {
                CollisionObjectType::Body => {
                    shape.collider_handle = collider_create_solid(
                        space_handle,
                        shape_handle,
                        &mat,
                        self.body_handle,
                        &user_data,
                    );
                }
                CollisionObjectType::Area => {
                    shape.collider_handle = collider_create_sensor(
                        space_handle,
                        shape_handle,
                        self.body_handle,
                        &user_data,
                    );
                }
            }

            assert!(is_handle_valid(shape.collider_handle));
            //self._init_collider(shape.collider_handle, space_handle);
        }
    }

    fn _destroy_shape(&mut self, shape: &mut CollisionObjectShape, p_shape_index: usize) {
        let mut shape_rid = Rid::Invalid;
        {
            let mut lock = spaces_singleton().lock().unwrap();
            if let Some(space) = lock.spaces.get_mut(&self.space) {
                let space_handle = space.get_handle();
                assert!(is_handle_valid(space_handle));
    
                assert!(is_handle_valid(shape.collider_handle));
    
                if self.area_detection_counter > 0 {
                    // Keep track of body information for delayed removal
                    space.add_removed_collider(shape.collider_handle, self.rid, self.instance_id, p_shape_index, self.get_type());
                }
    
                collider_destroy(space_handle, shape.collider_handle);
                shape_rid = shape.shape;
                shape.collider_handle = invalid_handle();
            }
        }
        {
            let mut lock = shapes_singleton().lock().unwrap();
            if let Some(shape) = lock.shapes.get_mut(&shape_rid) {
                shape.get_mut_base().destroy_rapier_shape();
            }
        }
    }

    fn update_shape_transform(&mut self, shape: &CollisionObjectShape) {
        let mut space_handle = invalid_handle();
        {
            let lock = spaces_singleton().lock().unwrap();
            if let Some(space) = lock.spaces.get(&self.space) {
                space_handle = space.get_handle();
            }
        }
        let origin = shape.xform.origin;
        let position = Vector::new(origin.x, origin.y);
        let angle = shape.xform.rotation();
        let scale = shape.xform.scale();

        assert!(is_handle_valid(space_handle));

        assert!(is_handle_valid(shape.collider_handle));
        let mut lock = shapes_singleton().lock().unwrap();
        if let Some(rapier_shape) = lock.shapes.get_mut(&shape.shape) {
            let shape_handle = rapier_shape.get_rapier_shape();
            assert!(is_handle_valid(shape_handle));
            let shape_info = ShapeInfo {
                handle: shape_handle,
                pixel_position: position,
                rotation: angle,
                skew: shape.xform.skew(),
                scale: Vector::new(scale.x, scale.y),
            };
            collider_set_transform(space_handle, shape.collider_handle, shape_info);
        }
    }

    pub fn update_transform(&mut self) {
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.space) {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(is_handle_valid(self.body_handle));

            let position = body_get_position(space_handle, self.body_handle);
            let angle = body_get_angle(space_handle, self.body_handle);
            let origin = Vector2::new(position.x, position.y);
            self.transform = Transform2D::from_angle_scale_skew_origin(angle, self.transform.scale(), self.transform.skew(), origin);

            self.inv_transform = self.transform.affine_inverse();
        }
    }
/*
    fn _init_material(&self, mat: &mut Material) {}

    fn _shapes_changed(&self) {}
*/
/*
    pub fn _set_space(&mut self, p_space: RapierSpace2D) {
        if let Some(space) = &self.space {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(is_handle_valid(self.body_handle));

            // This call also destroys the colliders
            body_destroy(space_handle, self.body_handle);
            self.body_handle = invalid_handle();

            for i in 0..self.shapes.len() {
                let shape = &mut self.shapes[i];
                if shape.disabled {
                    continue;
                }

                self._destroy_shape(shape, i as u32);
            }

            // Reset area detection counter to keep it consistent for new detections
            self.area_detection_counter = 0;
        }

        self.space = Some(p_space);

        if let Some(space) = &self.space {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(!is_handle_valid(self.body_handle));

            let user_data = UserData::default();
            self.set_body_user_data(&user_data);

            let position =
                Vector::new(self.transform.get_origin().x, self.transform.get_origin().y);
            let angle = self.transform.get_rotation();
            if self.mode == PhysicsServer2D::BODY_MODE_STATIC {
                self.body_handle =
                    body_create(space_handle, &position, angle, &user_data, BodyType::Static);
            } else if self.mode == PhysicsServer2D::BODY_MODE_KINEMATIC {
                self.body_handle = body_create(
                    space_handle,
                    &position,
                    angle,
                    &user_data,
                    BodyType::Kinematic,
                );
            } else {
                self.body_handle = body_create(
                    space_handle,
                    &position,
                    angle,
                    &user_data,
                    BodyType::Dynamic,
                );
            }

            for i in 0..self.shapes.len() {
                let shape = &mut self.shapes[i];
                if shape.disabled {
                    continue;
                }

                self._create_shape(shape, i as u32);
                self._update_shape_transform(shape);
            }
        }
    }
*/
    pub fn set_rid(&mut self, p_rid: Rid) {
        self.rid = p_rid;
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn set_instance_id(&mut self, p_instance_id: u64) {
        self.instance_id = p_instance_id;
    }

    pub fn get_instance_id(&self) -> u64 {
        self.instance_id
    }

    pub fn get_body_handle(&self) -> Handle {
        self.body_handle
    }

    pub fn set_canvas_instance_id(&mut self, p_canvas_instance_id: u64) {
        self.canvas_instance_id = p_canvas_instance_id;
    }

    pub fn get_canvas_instance_id(&self) -> u64 {
        self.canvas_instance_id
    }

    pub fn set_body_user_data(&self, r_user_data: &mut UserData) {
        r_user_data.part1 = self.rid.to_u64();
    }
    pub fn get_body_user_data(user_data: &UserData) -> Rid {
        let (rid, _) = Self::get_collider_user_data(user_data);
        return rid;
    }

    pub fn set_collider_user_data(&self, r_user_data: &mut UserData, p_shape_index: usize) {
        r_user_data.part1 = self.rid.to_u64();
        r_user_data.part2 = p_shape_index as u64;
    }

    pub fn get_collider_user_data(p_user_data: &UserData) -> (Rid, usize) {
        return (Rid::new(p_user_data.part1), p_user_data.part2 as usize);
    }
/*
    pub fn shape_changed(&self, p_shape: Rid) {
        if (!self.space.is_valid()) {
            return;
        }
        for i in 0..self.shapes.len() {
            let shape = self.shapes[i];
            if (shape.shape != p_shape) {
                continue;
            }
            if (shape.disabled) {
                continue;
            }

            self._destroy_shape(&shape, i);

            self._create_shape(&shape, i);
            self._update_shape_transform(&shape);
        }

        _shapes_changed();
    }
 */
    pub fn get_type(&self) -> CollisionObjectType {
        self.collision_object_type
    }
/*
    pub fn add_shape(
        &mut self,
        p_shape: Box<dyn IRapierShape2D>,
        p_transform: Transform2D,
        p_disabled: bool,
    ) {
        let shape = CollisionObjectShape {
            xform: p_transform,
            shape: p_shape.get_base().get_rid(),
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: invalid_handle(),
        };

        if !shape.disabled {
            self.create_shape(&shape, self.shapes.len());
            self.update_shape_transform(&shape);
        }

        self.shapes.push(shape);
        p_shape.add_owner(self.get_rid());

        if let Some(space) = &self.space {
            self._shapes_changed();
        }
    }
 */
/*
    pub fn set_shape(&mut self, p_index: usize, p_shape: RapierShape2D) {
        assert!(p_index < self.shapes.len());

        let shape = &mut self.shapes[p_index];

        self._destroy_shape(shape, p_index as u32);

        shape.shape.remove_owner(self);
        shape.shape = p_shape;

        p_shape.add_owner(self);

        if !shape.disabled {
            self._create_shape(shape, p_index as u32);
            self._update_shape_transform(shape);
        }

        if let Some(space) = &self.space {
            self._shapes_changed();
        }
    }

    pub fn set_shape_transform(&mut self, p_index: usize, p_transform: Transform2D) {
        assert!(p_index < self.shapes.len());

        let shape = &mut self.shapes[p_index];
        shape.xform = p_transform;

        self._update_shape_transform(shape);

        if let Some(space) = &self.space {
            self._shapes_changed();
        }
    }
*/
    pub fn get_shape_count(&self) -> i32 {
        return self.shapes.len() as i32;
    }

    pub fn get_shape(&self, idx: usize) -> Rid {
        return self.shapes[idx].shape;
    }

    pub fn get_shape_transform(&self, idx: usize) -> &Transform2D {
        &self.shapes[idx].xform
    }

    pub fn set_transform(&mut self, p_transform: Transform2D, wake_up: bool) {
        self.transform = p_transform;
        self.inv_transform = self.transform.affine_inverse();
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.space) {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(is_handle_valid(self.body_handle));

            let origin = self.transform.origin;
            let position = Vector::new(origin.x, origin.y);
            let rotation = self.transform.rotation();
            body_set_transform(space_handle, self.body_handle, &position, rotation, wake_up);
        }
    }

    pub fn get_transform(&self) -> Transform2D {
        self.transform
    }

    pub fn get_inv_transform(&self) -> Transform2D {
        self.inv_transform
    }

    pub fn get_space(&self) -> Rid {
        self.space
    }

/*
    pub fn set_shape_disabled(&mut self, p_index: usize, p_disabled: bool) {
        assert!(p_index < self.shapes.len());

        let shape = &mut self.shapes[p_index];
        if shape.disabled == p_disabled {
            return;
        }

        shape.disabled = p_disabled;

        if shape.disabled {
            self._destroy_shape(shape, p_index as u32);
        } else {
            self._create_shape(shape, p_index as u32);
            self._update_shape_transform(shape);
        }

        if let Some(space) = &self.space {
            self._shapes_changed();
        }
    }
 */
    pub fn is_shape_disabled(&self, idx: usize) -> bool {
        self.shapes[idx].disabled
    }

    pub fn set_shape_as_one_way_collision(
        &mut self,
        p_idx: usize,
        p_one_way_collision: bool,
        p_margin: real,
    ) {
        assert!(p_idx < self.shapes.len());

        let shape = &mut self.shapes[p_idx];
        shape.one_way_collision = p_one_way_collision;
        shape.one_way_collision_margin = p_margin;
    }

    pub fn is_shape_set_as_one_way_collision(&self, p_idx: usize) -> bool {
        assert!(p_idx < self.shapes.len());

        self.shapes[p_idx].one_way_collision
    }

    pub fn get_shape_one_way_collision_margin(&self, p_idx: usize) -> real {
        assert!(p_idx < self.shapes.len());

        self.shapes[p_idx].one_way_collision_margin
    }

    pub fn set_collision_mask(&mut self, p_mask: u32) {
        self.collision_mask = p_mask;
    }

    pub fn get_collision_mask(&self) -> u32 {
        self.collision_mask
    }

    pub fn set_collision_layer(&mut self, p_layer: u32) {
        self.collision_layer = p_layer;
    }

    pub fn get_collision_layer(&self) -> u32 {
        self.collision_layer
    }

    pub fn set_collision_priority(&mut self, p_priority: real) {
        assert!(p_priority > 0.0);
        self.collision_priority = p_priority;
    }

    pub fn get_collision_priority(&self) -> real {
        self.collision_priority
    }
/*
    pub fn remove_shape_rid(&mut self, shape: Rid) {
        // remove a shape, all the times it appears
        let mut i = 0;
        while i < self.shapes.len() {
            if self.shapes[i].shape == shape {
                self.remove_shape_idx(i);
            } else {
                i += 1;
            }
        }
    }

    pub fn remove_shape_idx(&mut self, p_index: usize) {
        // remove anything from shape to be erased to end, so subindices don't change
        assert!(p_index < self.shapes.len());

        let shape = &mut self.shapes[p_index];

        if !shape.disabled {
            self._destroy_shape(shape, p_index as u32);
        }

        shape.shape.remove_owner(self);
        self.shapes.remove(p_index);

        if let Some(space) = &self.space {
            self._shapes_changed();
        }
    }
 */
    pub fn _set_space(&mut self, space: Rid) {
        self.space = space;
    }

    pub fn set_pickable(&mut self, p_pickable: bool) {
        self.pickable = p_pickable;
    }

    pub fn is_pickable(&self) -> bool {
        self.pickable
    }

    pub fn collides_with(&self, p_other: &RapierCollisionObject2D) -> bool {
        p_other.collision_layer & self.collision_mask != 0
    }

    pub fn interacts_with(&self, p_other: &RapierCollisionObject2D) -> bool {
        self.collision_layer & p_other.collision_mask != 0
            || p_other.collision_layer & self.collision_mask != 0
    }
}
