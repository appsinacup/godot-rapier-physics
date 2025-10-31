use rapier::geometry::ColliderHandle;

use super::rapier_collision_object::IRapierCollisionObject;
use super::rapier_collision_object_base::CollisionObjectShape;
use super::rapier_collision_object_base::RapierCollisionObjectBase;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::servers::rapier_physics_singleton::PhysicsIds;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::PhysicsSpaces;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::servers::rapier_physics_singleton::get_id_rid;
use crate::shapes::rapier_shape::IRapierShape;
use crate::types::Transform;
impl RapierCollisionObjectBase {
    pub(super) fn recreate_shapes(
        collision_object: &mut dyn IRapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
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
                        physics_ids,
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
        p_shape_id: RapierId,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        let mut shape = CollisionObjectShape {
            xform: p_transform,
            id: p_shape_id,
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
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(p_shape_id, physics_ids)) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_id());
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }

    pub(super) fn shape_changed(
        collision_object: &mut dyn IRapierCollisionObject,
        shape_id: RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        for i in 0..collision_object.get_base().state.shapes.len() {
            let shape = collision_object.get_base().state.shapes[i];
            if shape.id != shape_id || shape.disabled {
                continue;
            }
            if collision_object.get_base().state.shapes[i].collider_handle
                != ColliderHandle::invalid()
            {
                collision_object.get_mut_base().state.shapes[i].collider_handle = collision_object
                    .get_base()
                    .destroy_shape(shape, i, physics_spaces, physics_engine, physics_ids);
            }
            collision_object.get_mut_base().state.shapes[i].collider_handle =
                collision_object.create_shape(
                    collision_object.get_base().state.shapes[i],
                    i,
                    physics_engine,
                );
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[i],
                physics_engine,
            );
        }
        collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
    }

    pub(super) fn remove_shape_idx(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
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
                physics_ids,
            );
        }
        let shape = collision_object.get_mut_base().state.shapes[p_index];
        collision_object.get_mut_base().state.shapes[p_index].collider_handle =
            ColliderHandle::invalid();
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(shape.id, physics_ids)) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_id());
        }
        collision_object.get_mut_base().state.shapes.remove(p_index);
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
        collision_object
            .get_mut_base()
            .update_shapes_indexes(physics_engine);
    }

    pub(super) fn set_shape(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_shape: RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
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
                physics_ids,
            );
        let shape = collision_object.get_base().state.shapes[p_index];
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(shape.id, physics_ids)) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_id());
        }
        collision_object.get_mut_base().state.shapes[p_index].id = p_shape;
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(shape.id, physics_ids)) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_id());
        }
        if !shape.disabled {
            collision_object.get_mut_base().state.shapes[p_index].collider_handle =
                collision_object.create_shape(
                    shape,
                    p_index,
                    physics_engine,
                );
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[p_index],
                physics_engine,
            );
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }

    pub(super) fn set_shape_transform(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
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
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }

    pub(super) fn set_shape_disabled(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
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
                    physics_ids,
                );
        }
        if !shape.disabled {
            collision_object.get_mut_base().state.shapes[p_index].collider_handle =
                collision_object.create_shape(
                    shape,
                    p_index,
                    physics_engine,
                );
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[p_index],
                physics_engine,
            );
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }
}
