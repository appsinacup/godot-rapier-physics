use rapier::prelude::*;
pub struct WorldSettings {
    pub particle_radius: Real,
    pub smoothing_factor: Real,
}
pub struct SimulationSettings {
    /// The timestep length (default: `1.0 / 60.0`)
    pub dt: Real,
    pub length_unit: Real,

    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of addition friction resolution iteration run during the last solver sub-step (default: `4`).
    pub num_additional_friction_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    pub pixel_gravity: Vector<Real>,
    pub pixel_liquid_gravity: Vector<Real>,
    pub max_ccd_substeps: usize,
}
