use crate::{joints::rapier_joint_base::{RapierJointBase, JointExport, JointImport}, shapes::rapier_shape_base::{RapierShapeBase, ShapeExport, ShapeImport}, spaces::rapier_space::{SpaceExport, SpaceImport}};
use crate::rapier_wrapper::prelude::PhysicsEngine;

use super::{rapier_area::{AreaExport, RapierArea, AreaImport}, rapier_body::{BodyExport, RapierBody, BodyImport}};

// The difference between Export and Import states is just that Exports contain non-owning references into state data
// (essentially they're just windows into the present state of a physics object). 
// Imports contain owned data pulled from serialized data, which can then be moved out into the objects.
#[cfg(feature = "serde-serialize")]
#[derive(serde::Serialize)]
pub enum ObjectExportState<'a> {
    RapierCollisionObject(AreaExport<'a>),
    RapierArea(AreaExport<'a>),
    RapierBody(BodyExport<'a>),
    RapierShapeBase(ShapeExport<'a>),
    RapierJointBase(JointExport<'a>),
    RapierSpace(SpaceExport<'a>),
}

#[cfg(feature = "serde-serialize")]
#[derive(serde::Deserialize)]
pub enum ObjectImportState {
    RapierCollisionObject(AreaImport),
    RapierArea(AreaImport),
    RapierBody(BodyImport),
    RapierShapeBase(ShapeImport),
    RapierJointBase(JointImport),
    RapierSpace(SpaceImport),
}

// pub enum ImportStateData {
//     RawState(ObjectImportState),
//     SerdeJson(serde_json::Value),
// }

// impl ImportStateData {
//     pub fn take_raw_state(self) -> ObjectImportState {
//         match self {
//             ImportStateData::RawState(r) => r,
//             _ => panic!("called into_raw_state() on non-rust ImportStateData"),
//         }
//     }
// }



#[cfg(feature = "serde-serialize")]
pub trait ExportableObject {    
    type ExportState<'a>: serde::Serialize where Self: 'a;
    
    //#[cfg(feature = "serde-serialize")]
    // Unfortunately, shapes require a physics engine reference to fetch their Rapier-side data.
    // I'm not 100% sure if this is necessary-- perhaps the space export can automatically rebuild shapes?
    fn get_export_state<'a>(&'a self, physics_engine: &'a mut crate::rapier_wrapper::prelude::PhysicsEngine) -> Option<Self::ExportState<'a>>;

    fn import_state(&mut self, physics_engine: &mut PhysicsEngine, data: ObjectImportState);

    //#[cfg(feature = "serde-serialize")]
    //fn import_json(&mut self, physics_engine: &mut PhysicsEngine, data: String);
}

// #[cfg(feature = "serde-serialize")]
// #[derive(Debug)]
// pub enum RapierPhysicsObject {
//     //RapierShapeBase(RapierShapeBase),
//     RapierArea(RapierArea),
//     RapierBody(RapierBody),
//     //RapierJointBase(RapierJointBase),
// }
// #[cfg(feature = "serde-serialize")]
// #[derive(Debug, serde::Serialize)]
// pub enum ObjectExportState<'a> {
//     //RapierShapeBase(ShapeExport<'a>),
//     RapierArea(AreaExport<'a>),
//     RapierBody(BodyExport<'a>),
//     //RapierJointBase(),
// }
// macro_rules! impl_rapier_exportable_object_trait {
//     ($enum_name:ident, $($variant:ident),*) => {
//         impl<'a> ExportableObject for $enum_name {
//             type ExportState<'b> = ObjectExportState<'b> where Self: 'b;

//             fn get_export_state(&self, physics_engine: &mut PhysicsEngine) -> Self::ExportState<'_> {
//                 match self {
//                     $(Self::$variant(co) => ObjectExportState::$variant(co.get_export_state(physics_engine)),)*
//                 }
//             }
//         }
//     };
// }
// impl_rapier_exportable_object_trait!(RapierPhysicsObject, RapierArea, RapierBody);