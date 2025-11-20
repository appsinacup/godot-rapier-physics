use super::rapier_area::AreaExport;
use super::rapier_area::AreaImport;
use super::rapier_body::BodyExport;
use super::rapier_body::BodyImport;
use crate::joints::rapier_joint_base::JointExport;
use crate::joints::rapier_joint_base::JointImport;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::shapes::rapier_shape_base::ShapeExport;
use crate::shapes::rapier_shape_base::ShapeImport;
use crate::spaces::rapier_space::SpaceExport;
use crate::spaces::rapier_space::SpaceImport;
// The difference between Export and Import states is just that Exports contain non-owning references into state data
// (essentially they're just windows into the present state of a physics object).
// Imports contain owned data pulled from serialized data, which can then be moved out into the objects.
#[cfg(feature = "serde-serialize")]
#[derive(serde::Serialize)]
pub enum ObjectExportState<'a> {
    Area(AreaExport<'a>),
    Body(BodyExport<'a>),
    ShapeBase(ShapeExport<'a>),
    JointBase(JointExport<'a>),
    Space(SpaceExport<'a>),
}
#[cfg(feature = "serde-serialize")]
#[derive(serde::Deserialize, Clone)]
pub enum ObjectImportState {
    Area(AreaImport),
    Body(BodyImport),
    ShapeBase(ShapeImport),
    JointBase(JointImport),
    Space(Box<SpaceImport>),
}
pub trait ExportToImport {
    fn into_import(self) -> ObjectImportState;
}
#[cfg(feature = "serde-serialize")]
pub trait ExportableObject {
    type ExportState<'a>: serde::Serialize
    where
        Self: 'a;
    //#[cfg(feature = "serde-serialize")]
    // Unfortunately, shapes require a physics engine reference to fetch their Rapier-side data.
    // I'm not 100% sure if this is necessary-- perhaps the space export can automatically rebuild shapes?
    fn get_export_state<'a>(
        &'a self,
        physics_engine: &'a mut crate::rapier_wrapper::prelude::PhysicsEngine,
    ) -> Option<Self::ExportState<'a>>;
    fn import_state(&mut self, physics_engine: &mut PhysicsEngine, data: ObjectImportState);
}
// This macro facilitates cloning of export types' internal state into owned copies, which are handed to a new import object.
macro_rules! impl_convert_to_import {
    ($export_enum:ident, $import_enum:ident, $($variant:ident),+) => {
        impl<'a> ExportToImport for $export_enum<'a> {
            fn into_import(self) -> $import_enum {
                match self {
                    $(
                        $export_enum::$variant(data) => {
                            $import_enum::$variant(data.into_import())
                        }
                    ),+
                }
            }
        }
    };
}
impl_convert_to_import!(
    ObjectExportState,
    ObjectImportState,
    Area,
    Body,
    ShapeBase,
    JointBase,
    Space
);
