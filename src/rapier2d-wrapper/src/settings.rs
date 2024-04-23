use rapier2d::prelude::*;
use crate::vector::Vector;

#[repr(C)]
pub struct WorldSettings {
	pub sleep_linear_threshold: Real,
	pub sleep_angular_threshold: Real,
	pub sleep_time_until_sleep: Real,
    pub max_ccd_substeps: usize,
    pub particle_radius: Real,
    pub smoothing_factor: Real,
}

#[no_mangle]
pub extern "C" fn default_world_settings() -> WorldSettings {
    WorldSettings {
		sleep_linear_threshold : 0.1,
		sleep_angular_threshold : 0.1,
		sleep_time_until_sleep : 1.0,
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
    pub pixel_gravity : Vector,
    pub pixel_liquid_gravity : Vector,
    pub max_ccd_substeps: usize,
}
