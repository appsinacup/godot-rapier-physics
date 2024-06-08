use nalgebra::Vector2;
use rapier::prelude::*;

pub struct WorldSettings {
    pub particle_radius: Real,
    pub smoothing_factor: Real,
}

impl WorldSettings {
    fn new(particle_radius: Real, smoothing_factor: Real) -> Self {
        Self {
            particle_radius,
            smoothing_factor,
        }
    }
}

pub struct SimulationSettings {
    /// The timestep length (default: `1.0 / 60.0`)
    pub dt: Real,

    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of addition friction resolution iteration run during the last solver sub-step (default: `4`).
    pub num_additional_friction_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    pub pixel_gravity: Vector2<Real>,
    pub pixel_liquid_gravity: Vector2<Real>,
    pub max_ccd_substeps: usize,
}
