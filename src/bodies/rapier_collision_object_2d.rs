use std::{cell::RefCell, rc::Rc};
use godot::{builtin::{Rid, Transform2D, Vector2}, engine::{native::ObjectId, physics_server_2d}};
use crate::{rapier2d::{body::{body_get_angle, body_get_position}, collider::Material, handle::{invalid_handle, is_handle_valid, Handle}}, shapes::rapier_shape_2d::IRapierShape2D, spaces::rapier_space_2d::RapierSpace2D};

pub trait IRapierCollisionObject2D {
    fn shape_changed(&self, shape: Rid);
    fn remove_shape(&self, shape: Rid);
}

#[derive(Clone, Copy, Debug)]
pub enum CollisionObjectType {
    Area,
    Body,
}

pub struct CollisionObjectShape {
    xform: Transform2D,
    shape: Option<Rc<RefCell<dyn IRapierShape2D>>>,
    disabled: bool,
    one_way_collision: bool,
    one_way_collision_margin: f32,
    collider_handle: Handle,
}

impl Default for CollisionObjectShape {
    fn default() -> Self {
        Self {
            xform: Transform2D::default(),
            shape: None,
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
    instance_id: ObjectId,
    canvas_instance_id: ObjectId,
    pickable: bool,
    shapes: Vec<CollisionObjectShape>,
    space: Option<Rc<RefCell<RapierSpace2D>>>,
    transform: Transform2D,
    inv_transform: Transform2D,
    collision_mask: u32,
    collision_layer: u32,
    collision_priority: f32,
    mode: physics_server_2d::BodyMode,
    body_handle: Handle,
    area_detection_counter: u32,
}

impl RapierCollisionObject2D {
    pub fn new(p_type: CollisionObjectType) -> Self {
        Self {
            collision_object_type: p_type,
            rid: Rid::Invalid,
            instance_id: ObjectId{id: 0},
            canvas_instance_id: ObjectId{id: 0},
            pickable: true,
            shapes: Vec::new(),
            space: None,
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

    pub fn set_rid(&mut self, p_rid: Rid) {
        self.rid = p_rid;
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn set_instance_id(&mut self, p_instance_id: ObjectId) {
        self.instance_id = p_instance_id;
    }

    pub fn get_instance_id(&self) -> ObjectId {
        self.instance_id.clone()
    }

    pub fn get_body_handle(&self) -> Handle {
        self.body_handle
    }

    pub fn set_canvas_instance_id(&mut self, p_canvas_instance_id: ObjectId) {
        self.canvas_instance_id = p_canvas_instance_id;
    }

    pub fn get_canvas_instance_id(&self) -> ObjectId {
        self.canvas_instance_id.clone()
    }
/*
    pub fn set_body_user_data(&self, r_user_data: &mut UserData) {
        r_user_data.part1 = self as *const _ as u64;
    }

    pub fn get_body_user_data(p_user_data: &UserData) -> &RapierCollisionObject2D {
        unsafe { &*(p_user_data.part1 as *const RapierCollisionObject2D) }
    }

    pub fn set_collider_user_data(&self, r_user_data: &mut UserData, p_shape_index: u32) {
        r_user_data.part1 = self.rid.to_u64();
        r_user_data.part2 = p_shape_index as u64;
    }

    pub fn get_collider_user_data(p_user_data: &UserData, r_shape_index: &mut u32) -> Rid {
        *r_shape_index = p_user_data.part2 as u32;
        return Rid::new(p_user_data.part1);
    }

    pub fn get_type(&self) -> CollisionObjectType {
        self.collision_object_type
    }

    pub fn add_shape(&mut self, p_shape: Rc<RefCell<dyn IRapierShape2D>>, p_transform: Transform2D, p_disabled: bool) {
        let shape = CollisionObjectShape {
            xform: p_transform,
            shape: Some(p_shape),
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: invalid_handle(),
        };

        if !shape.disabled {
            self._create_shape(&shape, self.shapes.len() as u32);
            self._update_shape_transform(&shape);
        }

        self.shapes.push(shape);
        p_shape.add_owner(self);

        if let Some(space) = &self.space {
            self._shapes_changed();
        }
    }

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

    pub fn remove_shape(&mut self, p_shape: RapierShape2D) {
        // remove a shape, all the times it appears
        let mut i = 0;
        while i < self.shapes.len() {
            if self.shapes[i].shape == p_shape {
                self.remove_shape(i);
            } else {
                i += 1;
            }
        }
    }

    pub fn remove_shape(&mut self, p_index: usize) {
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

    pub fn set_transform(&mut self, p_transform: Transform2D, wake_up: bool) {
        self.transform = p_transform;
        self.inv_transform = self.transform.affine_inverse();

        if let Some(space) = &self.space {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(is_handle_valid(self.body_handle));

            let origin = self.transform.get_origin();
            let position = Vector::new(origin.x, origin.y);
            let rotation = self.transform.get_rotation();
            body_set_transform(space_handle, self.body_handle, &position, rotation, wake_up);
        }
    }

    pub fn get_transform(&self) -> Transform2D {
        self.transform
    }

    pub fn get_inv_transform(&self) -> Transform2D {
        self.inv_transform
    }

    pub fn get_space(&self) -> Option<RapierSpace2D> {
        self.space
    }

    pub fn set_shape_as_one_way_collision(&mut self, p_idx: usize, p_one_way_collision: bool, p_margin: f32) {
        assert!(p_idx < self.shapes.len());

        let shape = &mut self.shapes[p_idx];
        shape.one_way_collision = p_one_way_collision;
        shape.one_way_collision_margin = p_margin;
    }

    pub fn is_shape_set_as_one_way_collision(&self, p_idx: usize) -> bool {
        assert!(p_idx < self.shapes.len());

        self.shapes[p_idx].one_way_collision
    }

    pub fn get_shape_one_way_collision_margin(&self, p_idx: usize) -> f32 {
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

    pub fn set_collision_priority(&mut self, p_priority: f32) {
        assert!(p_priority > 0.0);
        self.collision_priority = p_priority;
    }

    pub fn get_collision_priority(&self) -> f32 {
        self.collision_priority
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
        self.collision_layer & p_other.collision_mask != 0 || p_other.collision_layer & self.collision_mask != 0
    }

    pub fn set_space(&mut self, p_space: RapierSpace2D) {
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

            let position = Vector::new(self.transform.get_origin().x, self.transform.get_origin().y);
            let angle = self.transform.get_rotation();
            if self.mode == PhysicsServer2D::BODY_MODE_STATIC {
                self.body_handle = body_create(space_handle, &position, angle, &user_data, BodyType::Static);
            } else if self.mode == PhysicsServer2D::BODY_MODE_KINEMATIC {
                self.body_handle = body_create(space_handle, &position, angle, &user_data, BodyType::Kinematic);
            } else {
                self.body_handle = body_create(space_handle, &position, angle, &user_data, BodyType::Dynamic);
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

    fn _unregister_shapes(&self) {}

    fn _update_transform(&mut self) {
        if let Some(space) = &self.space {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(is_handle_valid(self.body_handle));

            let position = body_get_position(space_handle, self.body_handle);
            let angle = body_get_angle(space_handle, self.body_handle);

            self.transform.set_origin(Vector2::new(position.x, position.y));
            self.transform.set_rotation(angle);

            self.inv_transform = self.transform.affine_inverse();
        }
    }

    fn _create_shape(&mut self, shape: &CollisionObjectShape, p_shape_index: u32) {
        if let Some(space) = &self.space {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(!is_handle_valid(shape.collider_handle));

            let mat = Material::default();
            self._init_material(&mat);

            let shape_handle = shape.shape.get_rapier_shape();
            assert!(is_handle_valid(shape_handle));

            let mut user_data = UserData::default();
            self.set_collider_user_data(&mut user_data, p_shape_index);

            match self.type {
                Type::Body => {
                    shape.collider_handle = collider_create_solid(space_handle, shape_handle, &mat, self.body_handle, &user_data);
                }
                Type::Area => {
                    shape.collider_handle = collider_create_sensor(space_handle, shape_handle, self.body_handle, &user_data);
                }
            }

            assert!(is_handle_valid(shape.collider_handle));
            self._init_collider(shape.collider_handle);
        }
    }

    fn _destroy_shape(&mut self, shape: &Shape, p_shape_index: u32) {
        if let Some(space) = &self.space {
            let space_handle = space.get_handle();
            assert!(is_handle_valid(space_handle));

            assert!(is_handle_valid(shape.collider_handle));

            if self.area_detection_counter > 0 {
                // Keep track of body information for delayed removal
                space.add_removed_collider(shape.collider_handle, self, p_shape_index);
            }

            collider_destroy(space_handle, shape.collider_handle);
            shape.collider_handle = invalid_handle(); // collider_handle = rapier ID
        }
    }

    fn _update_shape_transform(&mut self, shape: &Shape) {
        if let Some(space) = &self.space {
            let space_handle = space.get_handle();

            let origin = shape.xform.get_origin();
            let position = Vector::new(origin.x, origin.y);
            let angle = shape.xform.get_rotation();

            assert!(is_handle_valid(space_handle));

            assert!(is_handle_valid(shape.collider_handle));
            assert!(is_handle_valid(shape.shape.get_rapier_shape()));
            let shape_info = ShapeInfo {
                shape: shape.shape.get_rapier_shape(),
                position,
                angle,
                skew: shape.xform.get_skew(),
                scale: Vector::new(shape.xform.get_scale().x, shape.xform.get_scale().y),
            };
            collider_set_transform(space_handle, shape.collider_handle, shape_info);
        }
    }

    fn _set_space(&mut self, p_space: RapierSpace2D) {
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

            let position = Vector::new(self.transform.get_origin().x, self.transform.get_origin().y);
            let angle = self.transform.get_rotation();
            if self.mode == PhysicsServer2D::BODY_MODE_STATIC {
                self.body_handle = body_create(space_handle, &position, angle, &user_data, BodyType::Static);
            } else if self.mode == PhysicsServer2D::BODY_MODE_KINEMATIC {
                self.body_handle = body_create(space_handle, &position, angle, &user_data, BodyType::Kinematic);
            } else {
                self.body_handle = body_create(space_handle, &position, angle, &user_data, BodyType::Dynamic);
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

    fn _shapes_changed(&self) {}

    fn _init_material(&self, mat: &mut Material) {}

    fn _init_collider(&self, collider_handle: Handle) {}
     */
}