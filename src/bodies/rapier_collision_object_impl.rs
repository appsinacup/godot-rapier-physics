use godot::builtin::Rid;
use rapier::geometry::ColliderHandle;

use super::rapier_collision_object::CollisionObjectShape;
use super::rapier_collision_object::IRapierCollisionObject;
use super::rapier_collision_object::RapierCollisionObject;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::PhysicsSpaces;
use crate::types::Transform;
impl RapierCollisionObject {
    pub(super) fn recreate_shapes(
        collision_object: &mut dyn IRapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        for i in 0..collision_object.get_base().get_shape_count() as usize {
            if collision_object.get_base().shapes[i].disabled {
                continue;
            }
            if collision_object.get_base().shapes[i].collider_handle != ColliderHandle::invalid() {
                collision_object.get_mut_base().shapes[i].collider_handle =
                    collision_object.get_base().destroy_shape(
                        collision_object.get_base().shapes[i],
                        i,
                        physics_spaces,
                        physics_engine,
                    );
            }
            collision_object.get_mut_base().shapes[i].collider_handle = collision_object
                .create_shape(
                    collision_object.get_base().shapes[i],
                    i,
                    physics_engine,
                    physics_shapes,
                );
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().shapes[i],
                physics_engine,
                physics_shapes,
            );
        }
    }

    pub(super) fn add_shape(
        collision_object: &mut dyn IRapierCollisionObject,
        p_shape: godot::prelude::Rid,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        let mut shape = CollisionObjectShape {
            xform: p_transform,
            shape: p_shape,
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: ColliderHandle::invalid(),
        };
        if !shape.disabled {
            shape.collider_handle = collision_object.create_shape(
                shape,
                collision_object.get_base().shapes.len(),
                physics_engine,
                physics_shapes,
            );
            collision_object.get_base().update_shape_transform(
                &shape,
                physics_engine,
                physics_shapes,
            );
        }
        collision_object.get_mut_base().shapes.push(shape);
        if let Some(shape) = physics_shapes.get_mut(&p_shape) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_rid());
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces);
        }
    }

    pub(super) fn shape_changed(
        collision_object: &mut dyn IRapierCollisionObject,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        for i in 0..collision_object.get_base().shapes.len() {
            let shape = collision_object.get_base().shapes[i];
            if shape.shape != p_shape || shape.disabled {
                continue;
            }
            if collision_object.get_base().shapes[i].collider_handle != ColliderHandle::invalid() {
                collision_object.get_mut_base().shapes[i].collider_handle = collision_object
                    .get_base()
                    .destroy_shape(shape, i, physics_spaces, physics_engine);
            }
            collision_object.get_mut_base().shapes[i].collider_handle =
                collision_object.get_base().create_shape(
                    collision_object.get_base().shapes[i],
                    i,
                    collision_object.init_material(),
                    physics_engine,
                    physics_shapes,
                );
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().shapes[i],
                physics_engine,
                physics_shapes,
            );
        }
        collision_object.shapes_changed(physics_engine, physics_spaces);
    }

    pub(super) fn remove_shape_idx(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        if p_index >= collision_object.get_base().shapes.len() {
            return;
        }
        let shape = &collision_object.get_base().shapes[p_index];
        if !shape.disabled {
            collision_object.get_base().destroy_shape(
                *shape,
                p_index,
                physics_spaces,
                physics_engine,
            );
        }
        let shape = &mut collision_object.get_mut_base().shapes[p_index];
        shape.collider_handle = ColliderHandle::invalid();
        if let Some(shape) = physics_shapes.get_mut(&shape.shape) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_rid());
        }
        collision_object.get_mut_base().shapes.remove(p_index);
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces);
        }
        collision_object
            .get_mut_base()
            .update_shapes_indexes(physics_engine);
    }

    pub(super) fn set_shape(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        if p_index >= collision_object.get_base().shapes.len() {
            return;
        }
        collision_object.get_mut_base().shapes[p_index].collider_handle =
            collision_object.get_base().destroy_shape(
                collision_object.get_base().shapes[p_index],
                p_index,
                physics_spaces,
                physics_engine,
            );
        let shape = collision_object.get_base().shapes[p_index];
        if let Some(shape) = physics_shapes.get_mut(&shape.shape) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_rid());
        }
        collision_object.get_mut_base().shapes[p_index].shape = p_shape;
        if let Some(shape) = physics_shapes.get_mut(&p_shape) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_rid());
        }
        if !shape.disabled {
            collision_object.get_mut_base().shapes[p_index].collider_handle =
                collision_object.get_base().create_shape(
                    shape,
                    p_index,
                    collision_object.init_material(),
                    physics_engine,
                    physics_shapes,
                );
            collision_object.get_base().update_shape_transform(
                &shape,
                physics_engine,
                physics_shapes,
            );
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces);
        }
    }

    pub(super) fn set_shape_transform(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        if p_index >= collision_object.get_base().shapes.len() {
            return;
        }
        collision_object.get_mut_base().shapes[p_index].xform = p_transform;
        let shape = &collision_object.get_base().shapes[p_index];
        collision_object
            .get_base()
            .update_shape_transform(shape, physics_engine, physics_shapes);
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces);
        }
    }

    pub(super) fn set_shape_disabled(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        if p_index >= collision_object.get_base().shapes.len() {
            return;
        }
        let shape = collision_object.get_base().shapes[p_index];
        if shape.disabled == p_disabled {
            return;
        }
        collision_object.get_mut_base().shapes[p_index].disabled = p_disabled;
        let shape = collision_object.get_base().shapes[p_index];
        if shape.disabled {
            collision_object.get_mut_base().shapes[p_index].collider_handle = collision_object
                .get_base()
                .destroy_shape(shape, p_index, physics_spaces, physics_engine);
        }
        if !shape.disabled {
            collision_object.get_mut_base().shapes[p_index].collider_handle =
                collision_object.get_base().create_shape(
                    shape,
                    p_index,
                    collision_object.init_material(),
                    physics_engine,
                    physics_shapes,
                );
            collision_object.get_base().update_shape_transform(
                &shape,
                physics_engine,
                physics_shapes,
            );
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces);
        }
    }
}
