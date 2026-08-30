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
#[cfg(feature = "dim2")]
use crate::types::Vector;
impl RapierCollisionObjectBase {
    pub(super) fn recreate_shapes(
        collision_object: &mut dyn IRapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        {
            if collision_object.get_base().is_compound() {
                // Still compound after the edit: swap the shape in place so the collider, and
                // every contact pair referencing it, survives.
                if collision_object
                    .get_base()
                    .wants_compound_collider(physics_engine)
                {
                    collision_object
                        .get_mut_base()
                        .update_compound_collider(physics_engine);
                    return;
                }
                // Falling back to one collider per shape: tear the compound down completely
                // first, a compound and per-shape colliders must never coexist on one object.
                collision_object.get_mut_base().destroy_shapes(
                    physics_engine,
                    physics_spaces,
                    physics_ids,
                );
                collision_object.get_mut_base().clear_compound();
            }
            if collision_object
                .get_base()
                .wants_compound_collider(physics_engine)
            {
                let built = Self::recreate_as_compound(
                    collision_object,
                    physics_engine,
                    physics_spaces,
                    physics_ids,
                );
                if built {
                    return;
                }
                // Refused once the shapes were already torn down. Fall through and give each its
                // own collider, rather than leave the object with none.
            }
        }
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
            let shape = collision_object.get_base().state.shapes[i];
            collision_object
                .get_mut_base()
                .update_shape_transform(i, shape, physics_engine);
        }
    }

    /// Rebuilds every shape of the object into one compound collider.
    ///
    /// The handle lives on the first enabled shape; the rest hold an invalid one, so the per-shape
    /// paths that key off a handle skip them and only this collider is destroyed later.
    ///
    /// Returns whether the compound was built. Scaling and skewing are applied here, after the
    /// per-shape colliders are torn down, and they can turn a shape into one that cannot be a
    /// compound part -- so a refusal arrives too late to avoid, and the caller has to put the
    /// object back on per-shape colliders rather than leave it with none.
    fn recreate_as_compound(
        collision_object: &mut dyn IRapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) -> bool {
        for i in 0..collision_object.get_base().get_shape_count() as usize {
            let shape = collision_object.get_base().state.shapes[i];
            if shape.collider_handle != ColliderHandle::invalid() {
                collision_object.get_mut_base().state.shapes[i].collider_handle = collision_object
                    .get_base()
                    .destroy_shape(shape, i, physics_spaces, physics_engine, physics_ids);
            }
        }

        let material = collision_object.init_material();
        let handle = collision_object
            .get_mut_base()
            .create_compound_collider(material, physics_engine);
        if handle == ColliderHandle::invalid() {
            collision_object.get_mut_base().clear_compound();
            return false;
        }

        let anchor = collision_object.get_base().compound_anchor_index();
        if let Some(anchor) = anchor {
            collision_object.get_mut_base().state.shapes[anchor].collider_handle = handle;
        }

        // Everything the compound did not take keeps a collider of its own, so one shape that
        // cannot be a part no longer costs the rest their place in the compound.
        for i in 0..collision_object.get_base().get_shape_count() as usize {
            let shape = collision_object.get_base().state.shapes[i];
            if shape.disabled || collision_object.get_base().is_compound_member(i) {
                continue;
            }
            let own = collision_object.create_shape(shape, i, physics_engine);
            collision_object.get_mut_base().state.shapes[i].collider_handle = own;
            collision_object
                .get_mut_base()
                .update_shape_transform(i, shape, physics_engine);
        }
        true
    }

    #[allow(clippy::too_many_arguments)]
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
        let shape = CollisionObjectShape {
            xform: p_transform,
            id: p_shape_id,
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            #[cfg(feature = "dim2")]
            one_way_collision_direction: Vector::new(0.0, 1.0),
            collider_handle: ColliderHandle::invalid(),
        };
        collision_object.get_mut_base().state.shapes.push(shape);
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(p_shape_id, physics_ids)) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_id());
        }
        // The shape that tips the object past one is what turns it into a compound; from then on
        // new shapes join the compound directly instead of getting a collider of their own.
        let joins_compound = collision_object.get_base().is_compound()
            || collision_object
                .get_base()
                .wants_compound_collider(physics_engine);
        if joins_compound {
            #[cfg(feature = "dim2")]
            Self::recreate_shapes(
                collision_object,
                physics_engine,
                physics_spaces,
                physics_ids,
            );
        } else if !p_disabled {
            let index = collision_object.get_base().state.shapes.len() - 1;
            let handle = collision_object.create_shape(shape, index, physics_engine);
            collision_object.get_mut_base().state.shapes[index].collider_handle = handle;
            let shape = collision_object.get_base().state.shapes[index];
            collision_object
                .get_mut_base()
                .update_shape_transform(index, shape, physics_engine);
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
        if collision_object.get_base().is_compound() {
            // The compound baked this shape's geometry, so any of its parts changing means a
            // rebuild.
            Self::recreate_shapes(
                collision_object,
                physics_engine,
                physics_spaces,
                physics_ids,
            );
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
            return;
        }
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
            collision_object.get_mut_base().state.shapes[i].collider_handle = collision_object
                .create_shape(
                    collision_object.get_base().state.shapes[i],
                    i,
                    physics_engine,
                );
            let shape = collision_object.get_base().state.shapes[i];
            collision_object
                .get_mut_base()
                .update_shape_transform(i, shape, physics_engine);
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
        if collision_object.get_base().is_compound() {
            let shape = collision_object.get_base().state.shapes[p_index];
            if let Some(shape) = physics_shapes.get_mut(&get_id_rid(shape.id, physics_ids)) {
                shape
                    .get_mut_base()
                    .remove_owner(collision_object.get_base().get_id());
            }
            collision_object.get_mut_base().state.shapes.remove(p_index);
            Self::recreate_shapes(
                collision_object,
                physics_engine,
                physics_spaces,
                physics_ids,
            );
            if collision_object.get_base().is_space_valid() {
                collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
            }
            return;
        }
        let shape = collision_object.get_base().state.shapes[p_index];
        if !shape.disabled {
            collision_object.get_base().destroy_shape(
                shape,
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
        if collision_object.get_base().is_compound() {
            let old_shape = collision_object.get_base().state.shapes[p_index];
            if let Some(shape) = physics_shapes.get_mut(&get_id_rid(old_shape.id, physics_ids)) {
                shape
                    .get_mut_base()
                    .remove_owner(collision_object.get_base().get_id());
            }
            collision_object.get_mut_base().state.shapes[p_index].id = p_shape;
            if let Some(shape) = physics_shapes.get_mut(&get_id_rid(p_shape, physics_ids)) {
                shape
                    .get_mut_base()
                    .add_owner(collision_object.get_base().get_id());
            }
            Self::recreate_shapes(
                collision_object,
                physics_engine,
                physics_spaces,
                physics_ids,
            );
            if collision_object.get_base().is_space_valid() {
                collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
            }
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
                collision_object.create_shape(shape, p_index, physics_engine);
            let shape = collision_object.get_base().state.shapes[p_index];
            collision_object
                .get_mut_base()
                .update_shape_transform(p_index, shape, physics_engine);
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
        let shape = collision_object.get_base().state.shapes[p_index];
        collision_object
            .get_mut_base()
            .update_shape_transform(p_index, shape, physics_engine);
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
        // Toggling a shape can also flip the whole object into or out of compound form.
        if collision_object.get_base().is_compound()
            || collision_object
                .get_base()
                .wants_compound_collider(physics_engine)
        {
            Self::recreate_shapes(
                collision_object,
                physics_engine,
                physics_spaces,
                physics_ids,
            );
            return;
        }
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
                collision_object.create_shape(shape, p_index, physics_engine);
            let shape = collision_object.get_base().state.shapes[p_index];
            collision_object
                .get_mut_base()
                .update_shape_transform(p_index, shape, physics_engine);
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }
}
