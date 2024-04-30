use crate::rapier2d::vector::Vector;
use rapier2d::prelude::*;

#[repr(C)]
pub struct WorldSettings {
    pub max_ccd_substeps: usize,
    pub particle_radius: Real,
    pub smoothing_factor: Real,
}

pub fn default_world_settings() -> WorldSettings {
    WorldSettings {
        max_ccd_substeps: 1,
        particle_radius: 0.1,
        smoothing_factor: 2.0,
    }
}

#[repr(C)]
pub struct SimulationSettings {
    /// The timestep length (default: `1.0 / 60.0`)
    pub dt: Real,

    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of addition friction resolution iteration run during the last solver sub-step (default: `4`).
    pub num_additional_friction_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    pub pixel_gravity: Vector,
    pub pixel_liquid_gravity: Vector,
    pub max_ccd_substeps: usize,
}
