use rapier::parry::query::ClosestPoints;
use rapier::parry::query::Contact;
use rapier::parry::query::ContactManifold as ParryContactManifold;
use rapier::parry::query::ContactManifoldsWorkspace;
use rapier::parry::query::DefaultQueryDispatcher;
use rapier::parry::query::NonlinearRigidMotion;
use rapier::parry::query::PersistentQueryDispatcher;
use rapier::parry::query::QueryDispatcher;
use rapier::parry::query::QueryDispatcherChain;
use rapier::parry::query::Ray;
use rapier::parry::query::ShapeCastHit;
use rapier::parry::query::ShapeCastOptions;
use rapier::parry::query::Unsupported;
use rapier::parry::query::details::NormalConstraints;
use rapier::parry::shape::PackedFeatureId;
use rapier::parry::shape::Segment;
use rapier::parry::shape::Shape;
use rapier::prelude::*;

use crate::rapier_wrapper::separation_ray::SEPARATION_RAY_AXIS;
use crate::rapier_wrapper::separation_ray::SeparationRayShape;

/// Must sit before [`DefaultQueryDispatcher`] in the chain: the default dispatcher does not
/// report `Unsupported` for a custom shape paired with a composite one (trimesh, heightfield,
/// polyline, compound), it descends into the composite and quietly returns no contact.
#[derive(Copy, Clone, Debug, Default)]
pub struct SeparationRayDispatcher;

pub fn separation_ray_query_dispatcher()
-> QueryDispatcherChain<SeparationRayDispatcher, DefaultQueryDispatcher> {
    SeparationRayDispatcher.chain(DefaultQueryDispatcher)
}

/// `pos12` is `g2`'s pose in `g1`'s local frame, and the result follows parry's convention:
/// `point1`/`normal1` live in `g1`'s frame, `point2`/`normal2` in `g2`'s.
fn contact_ray_shape(
    pos12: &Pose,
    ray: &SeparationRayShape,
    g2: &dyn Shape,
    prediction: Real,
) -> Option<Contact> {
    let axis = SEPARATION_RAY_AXIS;
    let query = Ray::new(Vector::ZERO, axis);
    // `solid` makes parry report a zero normal, dropped below, when the origin is buried in
    // `g2`. Casting non-solid instead returns the exit face and fires the body out through
    // the geometry.
    let hit = g2.cast_ray_and_get_normal(pos12, &query, ray.length + prediction, true)?;
    let surface_normal = hit.normal.try_normalize()?;
    if surface_normal.dot(axis) >= 0.0 {
        return None;
    }
    let dist = hit.time_of_impact - ray.length;
    let point1 = ray.tip();
    let normal1 = if ray.slide_on_slope {
        -surface_normal
    } else {
        axis
    };
    let point2_1 = point1 + normal1 * dist;
    Some(Contact::new(
        point1,
        pos12.inverse_transform_point(point2_1),
        normal1,
        -(pos12.rotation.inverse() * normal1),
        dist,
    ))
}
/// `None` when neither shape is a ray, which callers turn into `Unsupported` so the rest of
/// the chain handles the pair.
fn contact_any_order(
    pos12: &Pose,
    g1: &dyn Shape,
    g2: &dyn Shape,
    prediction: Real,
) -> Option<Option<Contact>> {
    if let Some(ray) = g1.as_shape::<SeparationRayShape>() {
        if g2.as_shape::<SeparationRayShape>().is_some() {
            return Some(None);
        }
        return Some(contact_ray_shape(pos12, ray, g2, prediction));
    }
    if let Some(ray) = g2.as_shape::<SeparationRayShape>() {
        return Some(contact_ray_shape(&pos12.inverse(), ray, g1, prediction).map(|c| c.flipped()));
    }
    None
}
fn segment_substitute(g: &dyn Shape) -> Option<Segment> {
    g.as_shape::<SeparationRayShape>().map(|ray| ray.segment())
}
impl QueryDispatcher for SeparationRayDispatcher {
    fn intersection_test(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<bool, Unsupported> {
        match contact_any_order(pos12, g1, g2, 0.0) {
            Some(contact) => Ok(contact.is_some_and(|c| c.dist <= 0.0)),
            None => Err(Unsupported),
        }
    }

    fn distance(&self, pos12: &Pose, g1: &dyn Shape, g2: &dyn Shape) -> Result<Real, Unsupported> {
        let (s1, s2) = (segment_substitute(g1), segment_substitute(g2));
        if s1.is_none() && s2.is_none() {
            return Err(Unsupported);
        }
        let g1 = s1.as_ref().map_or(g1, |s| s as &dyn Shape);
        let g2 = s2.as_ref().map_or(g2, |s| s as &dyn Shape);
        DefaultQueryDispatcher.distance(pos12, g1, g2)
    }

    fn contact(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: Real,
    ) -> Result<Option<Contact>, Unsupported> {
        contact_any_order(pos12, g1, g2, prediction).ok_or(Unsupported)
    }

    fn closest_points(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        max_dist: Real,
    ) -> Result<ClosestPoints, Unsupported> {
        let (s1, s2) = (segment_substitute(g1), segment_substitute(g2));
        if s1.is_none() && s2.is_none() {
            return Err(Unsupported);
        }
        let g1 = s1.as_ref().map_or(g1, |s| s as &dyn Shape);
        let g2 = s2.as_ref().map_or(g2, |s| s as &dyn Shape);
        DefaultQueryDispatcher.closest_points(pos12, g1, g2, max_dist)
    }

    /// Rays are never swept, which keeps them out of CCD and out of shape-cast queries.
    fn cast_shapes(
        &self,
        _pos12: &Pose,
        _local_vel12: Vector,
        g1: &dyn Shape,
        g2: &dyn Shape,
        _options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        if g1.as_shape::<SeparationRayShape>().is_some()
            || g2.as_shape::<SeparationRayShape>().is_some()
        {
            return Ok(None);
        }
        Err(Unsupported)
    }

    fn cast_shapes_nonlinear(
        &self,
        _motion1: &NonlinearRigidMotion,
        g1: &dyn Shape,
        _motion2: &NonlinearRigidMotion,
        g2: &dyn Shape,
        _start_time: Real,
        _end_time: Real,
        _stop_at_penetration: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        if g1.as_shape::<SeparationRayShape>().is_some()
            || g2.as_shape::<SeparationRayShape>().is_some()
        {
            return Ok(None);
        }
        Err(Unsupported)
    }
}
impl<ManifoldData, ContactData> PersistentQueryDispatcher<ManifoldData, ContactData>
    for SeparationRayDispatcher
where
    ManifoldData: Default,
    ContactData: Default + Copy,
{
    fn contact_manifolds(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: Real,
        manifolds: &mut Vec<ParryContactManifold<ManifoldData, ContactData>>,
        _workspace: &mut Option<ContactManifoldsWorkspace>,
    ) -> Result<(), Unsupported> {
        if contact_any_order(pos12, g1, g2, prediction).is_none() {
            return Err(Unsupported);
        }
        // A ray yields one contact point, so the manifold is reused to keep warm-start data.
        if manifolds.is_empty() {
            manifolds.push(ParryContactManifold::new());
        }
        manifolds.truncate(1);
        self.contact_manifold_convex_convex(
            pos12,
            g1,
            g2,
            None,
            None,
            prediction,
            &mut manifolds[0],
        )
    }

    fn contact_manifold_convex_convex(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        _normal_constraints1: Option<&dyn NormalConstraints>,
        _normal_constraints2: Option<&dyn NormalConstraints>,
        prediction: Real,
        manifold: &mut ParryContactManifold<ManifoldData, ContactData>,
    ) -> Result<(), Unsupported> {
        let Some(contact) = contact_any_order(pos12, g1, g2, prediction) else {
            return Err(Unsupported);
        };
        let Some(contact) = contact else {
            manifold.points.clear();
            return Ok(());
        };
        let fid = PackedFeatureId::face(0);
        let tracked = TrackedContact::new(contact.point1, contact.point2, fid, fid, contact.dist);
        if let Some(existing) = manifold.points.first_mut() {
            existing.copy_geometry_from(tracked);
        } else {
            manifold.points.push(tracked);
        }
        manifold.local_n1 = contact.normal1;
        manifold.local_n2 = contact.normal2;
        Ok(())
    }
}
#[cfg(all(test, feature = "dim2"))]
mod tests {
    use rapier::parry::shape::Cuboid;
    use rapier::parry::shape::HalfSpace;

    use super::*;

    const TOLERANCE: Real = 1.0e-5;
    const RAY_LENGTH: Real = 10.0;
    const FLOOR_HALF_THICKNESS: Real = 1.0;
    const SLOPE_NORMAL: Vector = Vector::new(0.6, -0.8);

    fn ray_contact(
        slide_on_slope: bool,
        g2: &dyn Shape,
        g2_origin: Vector,
        prediction: Real,
    ) -> Option<Contact> {
        let ray = SeparationRayShape::new(RAY_LENGTH, slide_on_slope);
        let pos12 = Pose::from_parts(g2_origin, Rotation::default());
        SeparationRayDispatcher
            .contact(&pos12, &ray, g2, prediction)
            .expect("a pair involving a separation ray is always supported")
    }

    fn floor_contact(surface_y: Real, prediction: Real) -> Option<Contact> {
        let floor = Cuboid::new(Vector::new(10.0, FLOOR_HALF_THICKNESS));
        let origin = Vector::new(0.0, surface_y + FLOOR_HALF_THICKNESS);
        ray_contact(false, &floor, origin, prediction)
    }

    fn slope_contact(surface_y: Real, slide_on_slope: bool) -> Option<Contact> {
        let slope = HalfSpace::new(SLOPE_NORMAL);
        ray_contact(slide_on_slope, &slope, Vector::new(0.0, surface_y), 0.0)
    }

    #[test]
    fn ray_does_not_reach_a_surface_past_its_tip() {
        assert!(floor_contact(RAY_LENGTH + 4.0, 0.0).is_none());
    }

    #[test]
    fn prediction_extends_the_ray() {
        let contact = floor_contact(RAY_LENGTH + 4.0, 5.0).expect("within prediction distance");
        assert!((contact.dist - 4.0).abs() < TOLERANCE);
    }

    #[test]
    fn tip_past_the_surface_reports_penetration_along_the_ray() {
        let contact = floor_contact(RAY_LENGTH - 2.0, 0.0).expect("the tip is past the surface");
        assert!((contact.dist + 2.0).abs() < TOLERANCE);
        assert!(contact.point1.distance(Vector::new(0.0, RAY_LENGTH)) < TOLERANCE);
        assert!(contact.normal1.distance(SEPARATION_RAY_AXIS) < TOLERANCE);
    }

    #[test]
    fn without_slide_on_slope_recovery_follows_the_ray_not_the_surface() {
        let contact = slope_contact(RAY_LENGTH - 2.0, false).expect("the tip is past the slope");
        assert!((contact.dist + 2.0).abs() < TOLERANCE);
        assert!(contact.normal1.distance(SEPARATION_RAY_AXIS) < TOLERANCE);
    }

    #[test]
    fn slide_on_slope_recovery_follows_the_surface_normal() {
        let contact = slope_contact(RAY_LENGTH - 2.0, true).expect("the tip is past the slope");
        assert!((contact.dist + 2.0).abs() < TOLERANCE);
        assert!(contact.normal1.distance(-SLOPE_NORMAL) < TOLERANCE);
        let witness = contact.point1 + contact.normal1 * contact.dist;
        assert!(witness.distance(contact.point1 + SLOPE_NORMAL * 2.0) < TOLERANCE);
    }

    #[test]
    fn a_ray_buried_in_a_halfspace_does_not_push() {
        let solid_below = HalfSpace::new(Vector::new(0.0, 1.0));
        assert!(ray_contact(false, &solid_below, Vector::new(0.0, 5.0), 0.0).is_none());
    }

    #[test]
    fn a_ray_buried_in_a_convex_shape_does_not_push() {
        let box_around_origin = Cuboid::new(Vector::new(5.0, 5.0));
        assert!(ray_contact(false, &box_around_origin, Vector::ZERO, 0.0).is_none());
    }

    #[test]
    fn a_back_face_cannot_push_the_body_out() {
        let facing_along_the_ray = HalfSpace::new(-SLOPE_NORMAL);
        assert!(ray_contact(false, &facing_along_the_ray, Vector::new(0.0, 8.0), 0.0).is_none());
    }

    #[test]
    fn two_rays_never_collide() {
        let other = SeparationRayShape::new(10.0, false);
        assert!(ray_contact(false, &other, Vector::new(0.0, 1.0), 0.0).is_none());
    }

    #[test]
    fn pairs_without_a_ray_are_left_to_the_rest_of_the_chain() {
        let floor = Cuboid::new(Vector::new(10.0, 1.0));
        let ball = rapier::parry::shape::Ball::new(1.0);
        let pos12 = Pose::from_parts(Vector::new(0.0, 1.0), Rotation::default());
        assert!(
            SeparationRayDispatcher
                .contact(&pos12, &ball, &floor, 0.0)
                .is_err()
        );
    }

    /// Goes through `shapes_contact`, the entry point the motion test's recovery pass uses.
    mod resting_position {
        use crate::rapier_wrapper::prelude::*;

        use super::*;

        const MARGIN: Real = 0.08;
        const RAY_LENGTH: Real = 30.0;
        const SURFACE_Y: Real = 450.0;
        const FLOOR_HALF_THICKNESS: Real = 100.0;

        const RAY_HANDLE: ShapeHandle = 1;
        const FLOOR_HANDLE: ShapeHandle = 2;

        fn engine_with_floor() -> PhysicsEngine {
            let mut engine = PhysicsEngine::default();
            engine.shape_create_separation_ray(RAY_LENGTH, false, RAY_HANDLE);
            engine.shape_create_box(
                Vector::new(1000.0, FLOOR_HALF_THICKNESS * 2.0),
                FLOOR_HANDLE,
            );
            engine
        }

        fn shape_at(handle: ShapeHandle, y: Real) -> ShapeInfo {
            ShapeInfo {
                handle,
                transform: Pose::from_parts(Vector::new(0.0, y), Rotation::default()),
                #[cfg(feature = "dim2")]
                skew: 0.0,
                scale: Vector::ONE,
            }
        }

        fn contact_with_ray_origin_at(y: Real) -> ContactResult {
            let engine = engine_with_floor();
            engine.shapes_contact(
                shape_at(RAY_HANDLE, y),
                shape_at(FLOOR_HANDLE, SURFACE_Y + FLOOR_HALF_THICKNESS),
                MARGIN,
            )
        }

        /// Mirrors the witness-point formula in `recover_motion_from_contacts`.
        fn recovery(contact: &ContactResult) -> Vector {
            let (a, b) = (contact.pixel_point1, contact.pixel_point2);
            let n = (a - b).normalize();
            -n * n.dot(a - b)
        }

        fn resting_origin_y() -> Real {
            SURFACE_Y - RAY_LENGTH
        }

        #[test]
        fn tip_exactly_on_the_surface_needs_no_recovery() {
            let contact = contact_with_ray_origin_at(resting_origin_y());
            assert!(contact.collided);
            assert!(recovery(&contact).length() <= MARGIN + TOLERANCE);
            assert!(contact.normal2.distance(Vector::new(0.0, -1.0)) < TOLERANCE);
        }

        #[test]
        fn a_sunk_body_is_pushed_back_up_by_the_penetration_depth() {
            const SUNK_BY: Real = 10.0;
            let contact = contact_with_ray_origin_at(resting_origin_y() + SUNK_BY);
            assert!(contact.collided);
            let recovery = recovery(&contact);
            assert!(recovery.x.abs() < TOLERANCE);
            assert!((recovery.y + SUNK_BY).abs() < MARGIN + TOLERANCE);
        }

        #[test]
        fn ray_reaches_no_further_than_its_length_plus_the_margin() {
            let contact = contact_with_ray_origin_at(resting_origin_y() - 1.0);
            assert!(!contact.collided);
        }

        #[test]
        fn ray_origin_below_the_surface_stops_pushing() {
            let contact = contact_with_ray_origin_at(SURFACE_Y + 1.0);
            assert!(!contact.collided);
        }
    }

    #[test]
    fn rays_are_never_swept() {
        let floor = Cuboid::new(Vector::new(10.0, 1.0));
        let ray = SeparationRayShape::new(10.0, false);
        let pos12 = Pose::from_parts(Vector::new(0.0, 15.0), Rotation::default());
        let hit = SeparationRayDispatcher
            .cast_shapes(
                &pos12,
                Vector::new(0.0, -10.0),
                &ray,
                &floor,
                ShapeCastOptions::default(),
            )
            .expect("a pair involving a separation ray is always supported");
        assert!(hit.is_none());
    }
}
