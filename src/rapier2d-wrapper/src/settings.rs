use rapier2d::prelude::*;
use crate::vector::Vector;

#[repr(C)]
pub struct WorldSettings {
	pub sleep_linear_threshold: Real,
	pub sleep_angular_threshold: Real,
	pub sleep_time_until_sleep: Real,
    pub solver_prediction_distance : Real,
}

#[no_mangle]
pub extern "C" fn default_world_settings() -> WorldSettings {
    WorldSettings {
		sleep_linear_threshold : 0.1,
		sleep_angular_threshold : 0.1,
		sleep_time_until_sleep : 1.0,
        solver_prediction_distance : 0.002,
    }
}

#[repr(C)]
pub struct SimulationSettings {
    /// The timestep length (default: `1.0 / 60.0`)
    pub dt: Real,

    /// 0-1: multiplier for how much of the constraint violation (e.g. contact penetration)
    /// will be compensated for during the velocity solve.
    /// (default `0.8`).
    pub erp: Real,
    /// 0-1: the damping ratio used by the springs for Baumgarte constraints stabilization.
    /// Lower values make the constraints more compliant (more "springy", allowing more visible penetrations
    /// before stabilization).
    /// (default `0.25`).
    pub damping_ratio: Real,

    /// 0-1: multiplier for how much of the joint violation
    /// will be compensated for during the velocity solve.
    /// (default `1.0`).
    pub joint_erp: Real,

    /// The fraction of critical damping applied to the joint for constraints regularization.
    /// (default `0.25`).
    pub joint_damping_ratio: Real,

    /// Amount of penetration the engine wont attempt to correct (default: `0.001m`).
    pub allowed_linear_error: Real,
    /// The maximal distance separating two objects that will generate predictive contacts (default: `0.002`).
    pub prediction_distance: Real,
    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of addition friction resolution iteration run during the last solver sub-step (default: `4`).
    pub num_additional_friction_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    pub pixel_gravity : Vector,
}
