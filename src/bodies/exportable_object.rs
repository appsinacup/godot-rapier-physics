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
#[allow(clippy::large_enum_variant)]
// Here we put the SpaceImport in a box; this is because Space state contains much more data than the other states,
// and Rust enums are sized depending on their largest member. Putting it in a box lets us refer to it as a pointer instead.
// Note that I've also told Clippy to allow large enum variants, which is because specifically in 3D,
// the BodyState is at least 448 bytes, which is twice the minimum size of AreaState.
// However, it's still pretty small, so I think this is fine.
pub enum ObjectImportState {
    Area(AreaImport),
    Body(BodyImport),
    ShapeBase(ShapeImport),
    JointBase(JointImport),
    Space(Box<SpaceImport>),
}
pub trait ExportToImport {
    type Import;
    fn into_import(self) -> Self::Import;
}
pub trait ImportToExport {
    type Export<'a>
    where
        Self: 'a;
    fn as_export<'a>(&'a self) -> Self::Export<'a>;
}
#[cfg(feature = "serde-serialize")]
pub trait ExportableObject {
    type ExportState<'a>: serde::Serialize
    where
        Self: 'a;
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
            type Import = $import_enum;
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

        impl ImportToExport for $import_enum {
            type Export<'a> = $export_enum<'a>;
            fn as_export<'a>(&'a self) -> $export_enum<'a> {
                match self {
                    $(
                        $import_enum::$variant(data) => {
                            $export_enum::$variant(data.as_export())
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
