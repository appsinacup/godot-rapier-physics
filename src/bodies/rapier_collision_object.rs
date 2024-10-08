use bodies::rapier_collision_object_base::CollisionObjectShape;
use bodies::rapier_collision_object_base::RapierCollisionObjectBase;
use godot::prelude::*;
use rapier::geometry::ColliderHandle;
use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::PhysicsShapes;
use servers::rapier_physics_singleton::PhysicsSpaces;
use servers::rapier_physics_singleton::RapierId;

use super::rapier_area::RapierArea;
use super::rapier_body::RapierBody;
use crate::rapier_wrapper::prelude::*;
use crate::types::*;
use crate::*;
pub trait IRapierCollisionObject: Sync {
    fn get_base(&self) -> &RapierCollisionObjectBase;
    fn get_mut_base(&mut self) -> &mut RapierCollisionObjectBase;
    fn get_body(&self) -> Option<&RapierBody>;
    fn get_area(&self) -> Option<&RapierArea>;
    fn get_mut_body(&mut self) -> Option<&mut RapierBody>;
    fn get_mut_area(&mut self) -> Option<&mut RapierArea>;
    fn set_space(
        &mut self,
        space: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &mut PhysicsIds,
    );
    fn recreate_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    );
    #[allow(clippy::too_many_arguments)]
    fn add_shape(
        &mut self,
        p_shape_id: RapierId,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    );
    fn set_shape(
        &mut self,
        shape_idx: usize,
        p_shape: ShapeHandle,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    );
    fn set_shape_transform(
        &mut self,
        shape_idx: usize,
        transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    );
    fn set_shape_disabled(
        &mut self,
        shape_idx: usize,
        disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    );
    fn remove_shape_idx(
        &mut self,
        p_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    );
    fn remove_shape_rid(
        &mut self,
        shape_rid: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    );
    fn create_shape(
        &mut self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        physics_engine: &mut PhysicsEngine,
    ) -> ColliderHandle;
    fn init_material(&self) -> Material;
    fn shapes_changed(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    );
    fn shape_changed(
        &mut self,
        shape_id: RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    );
    #[cfg(feature = "serde-serialize")]
    fn export_json(&self) -> String;
    #[cfg(feature = "serde-serialize")]
    fn export_binary(&self) -> PackedByteArray;
    #[cfg(feature = "serde-serialize")]
    fn import_binary(&mut self, data: PackedByteArray);
}
// TODO investigate large enum variant
#[allow(clippy::large_enum_variant)]
#[derive(Debug)]
pub enum RapierCollisionObject {
    RapierArea(RapierArea),
    RapierBody(RapierBody),
}
macro_rules! impl_rapier_collision_object_trait {
    ($enum_name:ident, $($variant:ident),*) => {
        impl IRapierCollisionObject for $enum_name {
            fn get_base(&self) -> &RapierCollisionObjectBase {
                match self {
                    $(Self::$variant(co) => co.get_base(),)*
                }
            }

            fn get_mut_base(&mut self) -> &mut RapierCollisionObjectBase {
                match self {
                    $(Self::$variant(co) => co.get_mut_base(),)*
                }
            }

            fn get_body(&self) -> Option<&RapierBody> {
                match self {
                    $(Self::$variant(co) => co.get_body(),)*
                }
            }

            fn get_area(&self) -> Option<&RapierArea> {
                match self {
                    $(Self::$variant(co) => co.get_area(),)*
                }
            }

            fn get_mut_body(&mut self) -> Option<&mut RapierBody> {
                match self {
                    $(Self::$variant(co) => co.get_mut_body(),)*
                }
            }

            fn get_mut_area(&mut self) -> Option<&mut RapierArea> {
                match self {
                    $(Self::$variant(co) => co.get_mut_area(),)*
                }
            }

            fn set_space(
                &mut self,
                space: Rid,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_ids: &mut PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.set_space(space, physics_engine, physics_spaces, physics_ids),)*
                }
            }

            fn recreate_shapes(
                &mut self,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.recreate_shapes(physics_engine, physics_spaces, physics_ids),)*
                }
            }

            fn add_shape(
                &mut self,
                p_shape_id: RapierId,
                p_transform: Transform,
                p_disabled: bool,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_shapes: &mut PhysicsShapes,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.add_shape(p_shape_id, p_transform, p_disabled, physics_engine, physics_spaces, physics_shapes, physics_ids),)*
                }
            }

            fn set_shape(
                &mut self,
                shape_idx: usize,
                p_shape: ShapeHandle,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_shapes: &mut PhysicsShapes,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.set_shape(shape_idx, p_shape, physics_engine, physics_spaces, physics_shapes, physics_ids),)*
                }
            }

            fn set_shape_transform(
                &mut self,
                shape_idx: usize,
                transform: Transform,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.set_shape_transform(shape_idx, transform, physics_engine, physics_spaces, physics_ids),)*
                }
            }

            fn set_shape_disabled(
                &mut self,
                shape_idx: usize,
                disabled: bool,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.set_shape_disabled(shape_idx, disabled, physics_engine, physics_spaces, physics_ids),)*
                }
            }

            fn remove_shape_idx(
                &mut self,
                p_index: usize,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_shapes: &mut PhysicsShapes,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.remove_shape_idx(p_index, physics_engine, physics_spaces, physics_shapes, physics_ids),)*
                }
            }

            fn remove_shape_rid(
                &mut self,
                shape_rid: Rid,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_shapes: &mut PhysicsShapes,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.remove_shape_rid(shape_rid, physics_engine, physics_spaces, physics_shapes, physics_ids),)*
                }
            }

            fn create_shape(
                &mut self,
                shape: CollisionObjectShape,
                p_shape_index: usize,
                physics_engine: &mut PhysicsEngine,
            ) -> ColliderHandle {
                match self {
                    $(Self::$variant(co) => co.create_shape(shape, p_shape_index, physics_engine),)*
                }
            }

            fn init_material(&self) -> Material {
                match self {
                    $(Self::$variant(co) => co.init_material(),)*
                }
            }

            fn shapes_changed(
                &mut self,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.shapes_changed(physics_engine, physics_spaces, physics_ids),)*
                }
            }

            fn shape_changed(
                &mut self,
                shape_id: RapierId,
                physics_engine: &mut PhysicsEngine,
                physics_spaces: &mut PhysicsSpaces,
                physics_ids: &PhysicsIds,
            ) {
                match self {
                    $(Self::$variant(co) => co.shape_changed(shape_id, physics_engine, physics_spaces, physics_ids),)*
                }
            }

            #[cfg(feature = "serde-serialize")]
            fn export_json(&self) -> String {
                match self {
                    $(Self::$variant(co) => co.export_json(),)*
                }
            }

            #[cfg(feature = "serde-serialize")]
            fn export_binary(&self) -> PackedByteArray {
                match self {
                    $(Self::$variant(co) => co.export_binary(),)*
                }
            }

            #[cfg(feature = "serde-serialize")]
            fn import_binary(
                &mut self,
                data: PackedByteArray,
            ) {
                match self {
                    $(Self::$variant(co) => co.import_binary(data),)*
                }
            }
        }
    };
}
impl_rapier_collision_object_trait!(RapierCollisionObject, RapierArea, RapierBody);
