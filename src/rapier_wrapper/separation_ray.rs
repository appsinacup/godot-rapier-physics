use rapier::parry::bounding_volume::BoundingSphere;
use rapier::parry::mass_properties::MassProperties;
use rapier::parry::query::PointProjection;
use rapier::parry::query::PointQuery;
use rapier::parry::query::Ray;
use rapier::parry::query::RayCast;
use rapier::parry::query::RayIntersection;
use rapier::parry::shape::FeatureId;
use rapier::parry::shape::Segment;
use rapier::parry::shape::Shape;
use rapier::parry::shape::TypedShape;
use rapier::prelude::*;

/// Matches Godot, which casts along local +Y in 2D and local +Z in 3D.
#[cfg(feature = "dim2")]
pub const SEPARATION_RAY_AXIS: Vector = Vector::new(0.0, 1.0);
#[cfg(feature = "dim3")]
pub const SEPARATION_RAY_AXIS: Vector = Vector::new(0.0, 0.0, 1.0);

/// Parry has no equivalent shape, so collisions are resolved by
/// [`SeparationRayDispatcher`](super::separation_ray_dispatcher::SeparationRayDispatcher).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SeparationRayShape {
    pub length: Real,
    pub slide_on_slope: bool,
}
impl SeparationRayShape {
    pub fn new(length: Real, slide_on_slope: bool) -> Self {
        Self {
            length,
            slide_on_slope,
        }
    }

    pub fn tip(&self) -> Vector {
        SEPARATION_RAY_AXIS * self.length
    }

    pub fn segment(&self) -> Segment {
        Segment::new(Vector::ZERO, self.tip())
    }
}
impl Shape for SeparationRayShape {
    fn compute_local_aabb(&self) -> Aabb {
        self.segment().local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere {
        self.segment().local_bounding_sphere()
    }

    fn clone_dyn(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn scale_dyn(&self, scale: Vector, _num_subdivisions: u32) -> Option<Box<dyn Shape>> {
        Some(Box::new(Self {
            length: self.length * SEPARATION_RAY_AXIS.dot(scale).abs(),
            slide_on_slope: self.slide_on_slope,
        }))
    }

    fn mass_properties(&self, _density: Real) -> MassProperties {
        MassProperties::default()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Custom
    }

    fn as_typed_shape(&self) -> TypedShape<'_> {
        TypedShape::Custom(self)
    }

    /// A body's CCD thickness is the minimum over its colliders, so reporting `MAX` keeps a
    /// shape with no volume from shrinking the substep size of the body carrying it.
    fn ccd_thickness(&self) -> Real {
        Real::MAX
    }

    fn ccd_angular_thickness(&self) -> Real {
        self.segment().ccd_angular_thickness()
    }
}
impl RayCast for SeparationRayShape {
    /// Godot's separation rays are invisible to ray queries.
    fn cast_local_ray_and_get_normal(
        &self,
        _ray: &Ray,
        _max_time_of_impact: Real,
        _solid: bool,
    ) -> Option<RayIntersection> {
        None
    }
}
impl PointQuery for SeparationRayShape {
    fn project_local_point(&self, pt: Vector, solid: bool) -> PointProjection {
        self.segment().project_local_point(pt, solid)
    }

    fn project_local_point_and_get_feature(&self, pt: Vector) -> (PointProjection, FeatureId) {
        self.segment().project_local_point_and_get_feature(pt)
    }
}
