
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::{joints::rapier_joint_base::{JointExport, JointImport}, shapes::rapier_shape_base::{ShapeExport, ShapeImport}, spaces::rapier_space::{SpaceExport, SpaceImport}};
use super::{rapier_area::{AreaExport, AreaImport}, rapier_body::{BodyExport, BodyImport}};

// The difference between Export and Import states is just that Exports contain non-owning references into state data
// (essentially they're just windows into the present state of a physics object). 
// Imports contain owned data pulled from serialized data, which can then be moved out into the objects.
#[cfg(feature = "serde-serialize")]
#[derive(serde::Serialize)]
pub enum ObjectExportState<'a> {
    RapierArea(AreaExport<'a>),
    RapierBody(BodyExport<'a>),
    RapierShapeBase(ShapeExport<'a>),
    RapierJointBase(JointExport<'a>),
    RapierSpace(SpaceExport<'a>),
}

#[cfg(feature = "serde-serialize")]
#[derive(serde::Deserialize, Clone)]
pub enum ObjectImportState {
    RapierArea(AreaImport),
    RapierBody(BodyImport),
    RapierShapeBase(ShapeImport),
    RapierJointBase(JointImport),
    RapierSpace(SpaceImport),
}

pub trait ExportToImport{
    fn to_import(self) -> ObjectImportState;
}

#[cfg(feature = "serde-serialize")]
pub trait ExportableObject {    
    type ExportState<'a>: serde::Serialize where Self: 'a;
    
    //#[cfg(feature = "serde-serialize")]
    // Unfortunately, shapes require a physics engine reference to fetch their Rapier-side data.
    // I'm not 100% sure if this is necessary-- perhaps the space export can automatically rebuild shapes?
    fn get_export_state<'a>(&'a self, physics_engine: &'a mut crate::rapier_wrapper::prelude::PhysicsEngine) -> Option<Self::ExportState<'a>>;

    fn import_state(&mut self, physics_engine: &mut PhysicsEngine, data: ObjectImportState);
}

// This macro facilitates cloning of export types' internal state into owned copies, which are handed to a new import object.
macro_rules! impl_convert_to_import {
    ($export_enum:ident, $import_enum:ident, $($variant:ident),+) => {
        impl<'a> ExportToImport for $export_enum<'a> {
            fn to_import(self) -> $import_enum {
                match self {
                    $(
                        $export_enum::$variant(data) => {
                            $import_enum::$variant(data.to_import())
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
    RapierArea,
    RapierBody,
    RapierShapeBase,
    RapierJointBase,
    RapierSpace
);