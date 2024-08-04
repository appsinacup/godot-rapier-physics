use rapier::prelude::*;
pub struct WorldSettings {
    /// - `particle_radius`: the radius of every particle for the fluid simulation.
    pub particle_radius: Real,
    /// - `smoothing_factor`: the smoothing factor used to compute the SPH kernel radius.
    ///    The kernel radius will be computed as `particle_radius * smoothing_factor * 2.0.
    pub smoothing_factor: Real,
    pub counters_enabled: bool,
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
    pub contact_damping_ratio: Real,
    pub contact_natural_frequency: Real,
    pub joint_damping_ratio: Real,
    pub joint_natural_frequency: Real,
    pub normalized_allowed_linear_error: Real,
    pub normalized_max_corrective_velocity: Real,
    pub normalized_prediction_distance: Real,
    pub num_internal_stabilization_iterations: usize,
}
