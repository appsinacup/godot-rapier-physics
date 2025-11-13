use crate::{joints::rapier_joint_base::{RapierJointBase, JointExport}, shapes::rapier_shape_base::{RapierShapeBase, ShapeExport}, spaces::rapier_space::SpaceExport};
use crate::rapier_wrapper::prelude::PhysicsEngine;

use super::{rapier_area::{AreaExport, RapierArea}, rapier_body::{BodyExport, RapierBody}};

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
pub trait ExportableObject {    
    type ExportState<'a>: serde::Serialize where Self: 'a;
    
    #[cfg(feature = "serde-serialize")]
    // Unfortunately, shapes require a physics engine reference to fetch their Rapier-side data.
    // I'm not 100% sure if this is necessary-- perhaps the space export can automatically rebuild shapes?
    fn get_export_state<'a>(&'a self, physics_engine: &'a mut crate::rapier_wrapper::prelude::PhysicsEngine) -> Option<Self::ExportState<'a>>;
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