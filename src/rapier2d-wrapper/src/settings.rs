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
    /// Minimum timestep size when using CCD with multiple substeps (default `1.0 / 60.0 / 100.0`)
    ///
    /// When CCD with multiple substeps is enabled, the timestep is subdivided
    /// into smaller pieces. This timestep subdivision won't generate timestep
    /// lengths smaller than `min_ccd_dt`.
    ///
    /// Setting this to a large value will reduce the opportunity to performing
    /// CCD substepping, resulting in potentially more time dropped by the
    /// motion-clamping mechanism. Setting this to an very small value may lead
    /// to numerical instabilities.
    pub min_ccd_dt: Real,

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
    /// Maximum amount of penetration the solver will attempt to resolve in one timestep.
    pub max_penetration_correction: Real,
    /// The maximal distance separating two objects that will generate predictive contacts (default: `0.002`).
    pub prediction_distance: Real,
    /// Maximum number of iterations performed to solve non-penetration and joint constraints (default: `4`).
    pub max_velocity_iterations: usize,
    /// Maximum number of iterations performed to solve friction constraints (default: `8`).
    pub max_velocity_friction_iterations: usize,
    /// Maximum number of iterations performed to remove the energy introduced by penetration corrections  (default: `1`).
    pub max_stabilization_iterations: usize,
    /// If `false`, friction and non-penetration constraints will be solved in the same loop. Otherwise,
    /// non-penetration constraints are solved first, and friction constraints are solved after (default: `true`).
    pub interleave_restitution_and_friction_resolution: bool,
    /// Minimum number of dynamic bodies in each active island (default: `128`).
    pub min_island_size: usize,
    /// Maximum number of substeps performed by the  solver (default: `1`).
    pub max_ccd_substeps: usize,

    pub gravity : Vector,
}
