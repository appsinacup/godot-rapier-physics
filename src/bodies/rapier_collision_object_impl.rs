use rapier::geometry::ColliderHandle;

use super::rapier_collision_object::IRapierCollisionObject;
use super::rapier_collision_object_base::CollisionObjectShape;
use super::rapier_collision_object_base::RapierCollisionObjectBase;
use crate::rapier_wrapper::handle::ShapeHandle;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::servers::rapier_physics_singleton::get_shape_rid;
use crate::servers::rapier_physics_singleton::PhysicsRids;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::PhysicsSpaces;
use crate::shapes::rapier_shape::IRapierShape;
use crate::types::Transform;
impl RapierCollisionObjectBase {
    pub(super) fn recreate_shapes(
        collision_object: &mut dyn IRapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        for i in 0..collision_object.get_base().get_shape_count() as usize {
            if collision_object.get_base().state.shapes[i].disabled {
                continue;
            }
            if collision_object.get_base().state.shapes[i].collider_handle
                != ColliderHandle::invalid()
            {
                collision_object.get_mut_base().state.shapes[i].collider_handle =
                    collision_object.get_base().destroy_shape(
                        collision_object.get_base().state.shapes[i],
                        i,
                        physics_spaces,
                        physics_engine,
                        physics_rids,
                    );
            }
            collision_object.get_mut_base().state.shapes[i].collider_handle = collision_object
                .create_shape(
                    collision_object.get_base().state.shapes[i],
                    i,
                    physics_engine,
                );
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[i],
                physics_engine,
            );
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn add_shape(
        collision_object: &mut dyn IRapierCollisionObject,
        p_shape: ShapeHandle,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        let mut shape = CollisionObjectShape {
            xform: p_transform,
            handle: p_shape,
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: ColliderHandle::invalid(),
        };
        if !shape.disabled {
            shape.collider_handle = collision_object.create_shape(
                shape,
                collision_object.get_base().state.shapes.len(),
                physics_engine,
            );
            collision_object
                .get_base()
                .update_shape_transform(&shape, physics_engine);
        }
        collision_object.get_mut_base().state.shapes.push(shape);
        if let Some(shape) = physics_shapes.get_mut(&get_shape_rid(p_shape, physics_rids)) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_body_handle());
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_rids);
        }
    }

    pub(super) fn shape_changed(
        collision_object: &mut dyn IRapierCollisionObject,
        old_shape_handle: ShapeHandle,
        new_shape_handle: ShapeHandle,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        for i in 0..collision_object.get_base().state.shapes.len() {
            let shape = collision_object.get_base().state.shapes[i];
            if shape.handle != old_shape_handle || shape.disabled {
                continue;
            }
            collision_object.get_mut_base().state.shapes[i].handle = new_shape_handle;
            if collision_object.get_base().state.shapes[i].collider_handle
                != ColliderHandle::invalid()
            {
                collision_object.get_mut_base().state.shapes[i].collider_handle = collision_object
                    .get_base()
                    .destroy_shape(shape, i, physics_spaces, physics_engine, physics_rids);
            }
            collision_object.get_mut_base().state.shapes[i].collider_handle =
                collision_object.get_base().create_shape(
                    collision_object.get_base().state.shapes[i],
                    i,
                    collision_object.init_material(),
                    physics_engine,
                );
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[i],
                physics_engine,
            );
        }
        collision_object.shapes_changed(physics_engine, physics_spaces, physics_rids);
    }

    pub(super) fn remove_shape_idx(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        let shape = &collision_object.get_base().state.shapes[p_index];
        if !shape.disabled {
            collision_object.get_base().destroy_shape(
                *shape,
                p_index,
                physics_spaces,
                physics_engine,
                physics_rids,
            );
        }
        let shape = &mut collision_object.get_mut_base().state.shapes[p_index];
        shape.collider_handle = ColliderHandle::invalid();
        if let Some(shape) = physics_shapes.get_mut(&get_shape_rid(shape.handle, physics_rids)) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_body_handle());
        }
        collision_object.get_mut_base().state.shapes.remove(p_index);
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_rids);
        }
        collision_object
            .get_mut_base()
            .update_shapes_indexes(physics_engine);
    }

    pub(super) fn set_shape(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_shape: ShapeHandle,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        collision_object.get_mut_base().state.shapes[p_index].collider_handle =
            collision_object.get_base().destroy_shape(
                collision_object.get_base().state.shapes[p_index],
                p_index,
                physics_spaces,
                physics_engine,
                physics_rids,
            );
        let shape = collision_object.get_base().state.shapes[p_index];
        if let Some(shape) = physics_shapes.get_mut(&get_shape_rid(shape.handle, physics_rids)) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_body_handle());
        }
        collision_object.get_mut_base().state.shapes[p_index].handle = p_shape;
        if let Some(shape) = physics_shapes.get_mut(&get_shape_rid(shape.handle, physics_rids)) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_body_handle());
        }
        if !shape.disabled {
            collision_object.get_mut_base().state.shapes[p_index].collider_handle =
                collision_object.get_base().create_shape(
                    shape,
                    p_index,
                    collision_object.init_material(),
                    physics_engine,
                );
            collision_object
                .get_base()
                .update_shape_transform(&shape, physics_engine);
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_rids);
        }
    }

    pub(super) fn set_shape_transform(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        collision_object.get_mut_base().state.shapes[p_index].xform = p_transform;
        let shape = &collision_object.get_base().state.shapes[p_index];
        collision_object
            .get_base()
            .update_shape_transform(shape, physics_engine);
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_rids);
        }
    }

    pub(super) fn set_shape_disabled(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        let shape = collision_object.get_base().state.shapes[p_index];
        if shape.disabled == p_disabled {
            return;
        }
        collision_object.get_mut_base().state.shapes[p_index].disabled = p_disabled;
        let shape = collision_object.get_base().state.shapes[p_index];
        if shape.disabled {
            collision_object.get_mut_base().state.shapes[p_index].collider_handle =
                collision_object.get_base().destroy_shape(
                    shape,
                    p_index,
                    physics_spaces,
                    physics_engine,
                    physics_rids,
                );
        }
        if !shape.disabled {
            collision_object.get_mut_base().state.shapes[p_index].collider_handle =
                collision_object.get_base().create_shape(
                    shape,
                    p_index,
                    collision_object.init_material(),
                    physics_engine,
                );
            collision_object
                .get_base()
                .update_shape_transform(&shape, physics_engine);
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_rids);
        }
    }
}
