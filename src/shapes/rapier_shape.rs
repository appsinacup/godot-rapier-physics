#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;

use super::rapier_capsule_shape::RapierCapsuleShape;
use super::rapier_circle_shape::RapierCircleShape;
use super::rapier_concave_polygon_shape::RapierConcavePolygonShape;
use super::rapier_convex_polygon_shape::RapierConvexPolygonShape;
#[cfg(feature = "dim3")]
use super::rapier_cylinder_shape_3d::RapierCylinderShape3D;
#[cfg(feature = "dim3")]
use super::rapier_heightmap_shape_3d::RapierHeightMapShape3D;
use super::rapier_rectangle_shape::RapierRectangleShape;
#[cfg(feature = "dim2")]
use super::rapier_segment_shape_2d::RapierSegmentShape2D;
use super::rapier_separation_ray_shape::RapierSeparationRayShape;
use super::rapier_world_boundary_shape::RapierWorldBoundaryShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsRids;
use crate::shapes::rapier_shape_base::RapierShapeBase;
pub trait IRapierShape {
    fn get_base(&self) -> &RapierShapeBase;
    fn get_mut_base(&mut self) -> &mut RapierShapeBase;
    fn get_type(&self) -> ShapeType;
    fn allows_one_way_collision(&self) -> bool;
    fn set_data(
        &mut self,
        data: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    );
    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant;
}
pub enum RapierShape {
    RapierCapsuleShape(RapierCapsuleShape),
    RapierCircleShape(RapierCircleShape),
    RapierConcavePolygonShape(RapierConcavePolygonShape),
    RapierConvexPolygonShape(RapierConvexPolygonShape),
    #[cfg(feature = "dim3")]
    RapierCylinderShape3D(RapierCylinderShape3D),
    #[cfg(feature = "dim3")]
    RapierHeightMapShape3D(RapierHeightMapShape3D),
    RapierRectangleShape(RapierRectangleShape),
    #[cfg(feature = "dim2")]
    RapierSegmentShape2D(RapierSegmentShape2D),
    RapierSeparationRayShape(RapierSeparationRayShape),
    RapierWorldBoundaryShape(RapierWorldBoundaryShape),
}
macro_rules! impl_rapier_shape_trait {
    ($enum_name:ident, $($variant:ident),*) => {
        impl IRapierShape for $enum_name {
            fn get_base(&self) -> &RapierShapeBase {
                match self {
                    $(Self::$variant(s) => s.get_base(),)*
                }
            }

            fn get_mut_base(&mut self) -> &mut RapierShapeBase {
                match self {
                    $(Self::$variant(s) => s.get_mut_base(),)*
                }
            }

            fn get_type(&self) -> ShapeType {
                match self {
                    $(Self::$variant(s) => s.get_type(),)*
                }
            }

            fn allows_one_way_collision(&self) -> bool {
                match self {
                    $(Self::$variant(s) => s.allows_one_way_collision(),)*
                }
            }

            fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine, physics_rids: &mut PhysicsRids) {
                match self {
                    $(Self::$variant(s) => s.set_data(data, physics_engine, physics_rids),)*
                }
            }

            fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
                match self {
                    $(Self::$variant(s) => s.get_data(physics_engine),)*
                }
            }
        }
    };
}
#[cfg(feature = "dim3")]
impl_rapier_shape_trait!(
    RapierShape,
    RapierCapsuleShape,
    RapierCircleShape,
    RapierConcavePolygonShape,
    RapierConvexPolygonShape,
    RapierCylinderShape3D,
    RapierHeightMapShape3D,
    RapierRectangleShape,
    RapierSeparationRayShape,
    RapierWorldBoundaryShape
);
#[cfg(feature = "dim2")]
impl_rapier_shape_trait!(
    RapierShape,
    RapierCapsuleShape,
    RapierCircleShape,
    RapierConcavePolygonShape,
    RapierConvexPolygonShape,
    RapierSegmentShape2D,
    RapierRectangleShape,
    RapierSeparationRayShape,
    RapierWorldBoundaryShape
);
