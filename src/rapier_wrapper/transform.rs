use rapier::math::{Isometry, Real, Vector};

pub struct Transform {
    pub isometry: Isometry<Real>,
    pub scale: Vector<Real>,
    #[cfg(feature = "dim2")]
    pub skew: Real,
}