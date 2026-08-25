use godot::classes::*;
use godot::prelude::*;
use godot::register::info::PropertyHint;
use rapier::dynamics::IntegrationParameters;
use rapier::math::Real;
/// Worker count for the solver: performance cores only.
///
/// A solver step is a chain of barrier-synchronised stages, so each stage runs at the speed
/// of its slowest worker. Workers on efficiency cores gate all the others, costing more than
/// the extra workers gain -- ~50% on a 4P+4E M1.
#[cfg(feature = "parallel")]
fn performance_core_count() -> usize {
    #[cfg(target_vendor = "apple")]
    if let Some(cores) = apple_performance_cores() {
        return cores;
    }
    #[cfg(target_os = "linux")]
    if let Some(cores) = linux_performance_cores() {
        return cores;
    }
    num_cpus::get_physical()
}

/// `perflevel0` is the fastest core class; Intel Macs report a single level, so every core.
#[cfg(all(feature = "parallel", target_vendor = "apple"))]
fn apple_performance_cores() -> Option<usize> {
    let mut cores: libc::c_uint = 0;
    let mut size = size_of_val(&cores);
    let rc = unsafe {
        libc::sysctlbyname(
            c"hw.perflevel0.logicalcpu".as_ptr(),
            (&raw mut cores).cast(),
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };
    (rc == 0 && cores > 0).then_some(cores as usize)
}

/// Hybrid Intel exposes performance cores as their own PMU, whose `cpus` file holds a list
/// like `0-15` or `0-7,20-23`. Absent on non-hybrid machines.
#[cfg(all(feature = "parallel", target_os = "linux"))]
fn linux_performance_cores() -> Option<usize> {
    let list = std::fs::read_to_string("/sys/devices/cpu_core/cpus").ok()?;
    let cores: usize = list
        .trim()
        .split(',')
        .filter_map(|range| {
            let (first, last) = range.split_once('-').unwrap_or((range, range));
            Some(last.trim().parse::<usize>().ok()? - first.trim().parse::<usize>().ok()? + 1)
        })
        .sum();
    (cores > 0).then_some(cores)
}
#[cfg(feature = "dim2")]
const RUN_ON_SEPARATE_THREAD: &str = "physics/2d/run_on_separate_thread";
#[cfg(feature = "dim3")]
const RUN_ON_SEPARATE_THREAD: &str = "physics/3d/run_on_separate_thread";
const SOLVER_NUM_ITERATIONS: &str = "physics/rapier/solver/num_iterations";
const SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS: &str =
    "physics/rapier/solver/num_internal_stabilization_iterations";
const SOLVER_NUM_INTERNAL_PGS_ITERATIONS: &str =
    "physics/rapier/solver/num_internal_pgs_iterations";
const SOLVER_MAX_CCD_SUBSTEPS: &str = "physics/rapier/solver/max_ccd_substeps";
const SOLVER_NORMALIZED_ALLOWED_LINEAR_ERROR: &str =
    "physics/rapier/solver/normalized_allowed_linear_error";
const SOLVER_NORMALIZED_MAX_CORRECTIVE_VELOCITY: &str =
    "physics/rapier/solver/normalized_max_corrective_velocity";
const SOLVER_NORMALIZED_PREDICTION_DISTANCE: &str =
    "physics/rapier/solver/normalized_prediction_distance";
const SOLVER_NORMALIZED_MAX_LINEAR_VELOCITY: &str =
    "physics/rapier/solver/normalized_max_linear_velocity";
const SOLVER_PREDICTIVE_CONTACT_ALLOWANCE_THRESHOLD: &str =
    "physics/rapier/solver/predictive_contact_allowance_threshold";
const CONTACT_DAMPING_RATIO: &str = "physics/rapier/solver/contact_damping_ratio";
const MOTION_RECOVER_ATTEMPTS: &str = "physics/rapier/motion/recover_attempts";
const MOTION_RECOVER_RATIO: &str = "physics/rapier/motion/recover_ratio";
const MOTION_CAST_ITERATIONS: &str = "physics/rapier/motion/cast_iterations";
#[cfg(feature = "dim2")]
const MOTION_STUCK_PENETRATION: &str = "physics/rapier/motion/stuck_penetration_threshold_2d";
#[cfg(feature = "dim3")]
const MOTION_STUCK_PENETRATION: &str = "physics/rapier/motion/stuck_penetration_threshold_3d";
const QUERY_MAX_SHAPE_CAST_RESULTS: &str = "physics/rapier/queries/max_shape_cast_results";
const QUERY_POINT_HONORS_PICKABLE: &str = "physics/rapier/queries/point_query_honors_pickable";
const SHAPE_SCALE_SUBDIVISIONS: &str = "physics/rapier/shapes/scale_subdivisions";
const MOTION_RECOVER_ATTEMPTS_DEFAULT: i32 = 4;
const MOTION_CAST_ITERATIONS_DEFAULT: i32 = 8;
const MAX_SHAPE_CAST_RESULTS_DEFAULT: usize = 64;
const SHAPE_SCALE_SUBDIVISIONS_DEFAULT: u32 = 20;
#[cfg(feature = "dim2")]
const MOTION_RECOVER_RATIO_DEFAULT: f64 = 0.4;
#[cfg(feature = "dim3")]
const MOTION_RECOVER_RATIO_DEFAULT: f64 = 0.5;
#[cfg(feature = "dim2")]
const MOTION_STUCK_PENETRATION_DEFAULT: f64 = 0.1;
#[cfg(feature = "dim3")]
const MOTION_STUCK_PENETRATION_DEFAULT: f64 = 0.01;
const CONTACT_NATURAL_FREQUENCY: &str = "physics/rapier/solver/contact_natural_frequency";
const DEFAULT_MAX_CCD_SUBSTEPS: i32 = 2;
const DEFAULT_PGS_ITERATIONS: i32 = 1;
const DEFAULT_STABILIZATION_ITERATIONS: i32 = 1;
// Softer contacts settle deeper under the weight of a pile and keep creeping instead of
// resting, so a low natural frequency or a damping ratio above rapier's default of 10 both
// leave stacks jittering. Values chosen by sweeping the 2D suite; see
// scripts/sweep-solver-params.sh to re-measure.
#[cfg(feature = "dim2")]
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier/fluid/fluid_particle_radius_2d";
#[cfg(feature = "dim3")]
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier/fluid/fluid_particle_radius_3d";
const FLUID_SMOOTHING_FACTOR: &str = "physics/rapier/fluid/fluid_smoothing_factor";
const FLUID_BOUNDARY_COEFF: &str = "physics/rapier/fluid/fluid_boundary_coefficient";
#[cfg(feature = "dim2")]
const LENGTH_UNIT: &str = "physics/rapier/solver/length_unit_2d";
#[cfg(feature = "dim2")]
const ORIENTED_CONCAVE_POLYLINE: &str = "physics/rapier/logic/oriented_concave_polyline_2d";
#[cfg(feature = "dim2")]
const LENGTH_UNIT_VALUE: real = 100.0;
#[cfg(feature = "dim2")]
const LENGTH_UNIT_HINT: &str = "1,100,1,suffix:length_unit,or_greater";
#[cfg(feature = "dim2")]
const FLUID_PARTICLE_VALUE: real = 20.0;
#[cfg(feature = "dim3")]
const FLUID_PARTICLE_VALUE: real = 0.2;
#[cfg(feature = "dim3")]
const LENGTH_UNIT: &str = "physics/rapier/solver/length_unit_3d";
#[cfg(feature = "dim3")]
const LENGTH_UNIT_VALUE: real = 1.0;
#[cfg(feature = "dim3")]
const LENGTH_UNIT_HINT: &str = "0.0001,1,0.0001,suffix:length_unit,or_greater";
/// Read once at startup; changing any of these needs a restart.
pub struct MotionSettings {
    pub recover_attempts: i32,
    pub recover_ratio: Real,
    pub cast_iterations: i32,
    pub stuck_penetration: Real,
    pub max_shape_cast_results: usize,
    pub point_query_honors_pickable: bool,
    pub shape_scale_subdivisions: u32,
}

impl Default for MotionSettings {
    fn default() -> Self {
        Self {
            recover_attempts: MOTION_RECOVER_ATTEMPTS_DEFAULT,
            recover_ratio: MOTION_RECOVER_RATIO_DEFAULT as Real,
            cast_iterations: MOTION_CAST_ITERATIONS_DEFAULT,
            stuck_penetration: MOTION_STUCK_PENETRATION_DEFAULT as Real,
            max_shape_cast_results: MAX_SHAPE_CAST_RESULTS_DEFAULT,
            point_query_honors_pickable: true,
            shape_scale_subdivisions: SHAPE_SCALE_SUBDIVISIONS_DEFAULT,
        }
    }
}

static MOTION_SETTINGS: std::sync::OnceLock<MotionSettings> = std::sync::OnceLock::new();

/// Set when the extension reaches `InitStage::Servers`, after which reading project settings is
/// safe. Stands in for `godot::sys::is_initialized()`, which gdext removed in 0.5.5 without a
/// public replacement; anything reached before that point has to fall back to defaults.
static BINDINGS_READY: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn mark_bindings_ready() {
    BINDINGS_READY.store(true, std::sync::atomic::Ordering::Relaxed);
}

fn bindings_ready() -> bool {
    BINDINGS_READY.load(std::sync::atomic::Ordering::Relaxed)
}

pub fn motion_settings() -> &'static MotionSettings {
    MOTION_SETTINGS.get_or_init(|| {
        if !bindings_ready() {
            return MotionSettings::default();
        }
        MotionSettings {
            recover_attempts: RapierProjectSettings::get_setting_int(MOTION_RECOVER_ATTEMPTS).max(1)
                as i32,
            recover_ratio: (RapierProjectSettings::get_setting_double(MOTION_RECOVER_RATIO)
                as Real)
                .clamp(0.05, 1.0),
            cast_iterations: RapierProjectSettings::get_setting_int(MOTION_CAST_ITERATIONS).max(1)
                as i32,
            stuck_penetration: RapierProjectSettings::get_setting_double(MOTION_STUCK_PENETRATION)
                as Real,
            max_shape_cast_results: RapierProjectSettings::get_setting_int(
                QUERY_MAX_SHAPE_CAST_RESULTS,
            )
            .max(1) as usize,
            point_query_honors_pickable: ProjectSettings::singleton()
                .get_setting_with_override(QUERY_POINT_HONORS_PICKABLE)
                .try_to()
                .unwrap_or(true),
            shape_scale_subdivisions: RapierProjectSettings::get_setting_int(
                SHAPE_SCALE_SUBDIVISIONS,
            )
            .max(4) as u32,
        }
    })
}

pub fn register_setting(
    p_name: &str,
    p_value: Variant,
    p_needs_restart: bool,
    p_hint: PropertyHint,
    p_hint_string: &str,
) {
    let mut project_settings = ProjectSettings::singleton();
    if !project_settings.has_setting(p_name) {
        project_settings.set(p_name, &p_value.clone());
    }
    let mut property_info = VarDictionary::new();
    let _ = property_info.insert("name", p_name);
    let _ = property_info.insert("type", p_value.get_type());
    let _ = property_info.insert("hint", p_hint);
    let _ = property_info.insert("hint_string", p_hint_string);
    project_settings.add_property_info(&property_info);
    project_settings.set_initial_value(p_name, &p_value);
    project_settings.set_restart_if_changed(p_name, p_needs_restart);
    static mut ORDER: i32 = 1000000;
    unsafe {
        project_settings.set_order(p_name, ORDER);
        ORDER += 1;
    }
}
pub fn register_setting_ranged(
    p_name: &str,
    p_value: Variant,
    p_hint_string: &str,
    p_needs_restart: bool,
) {
    register_setting(
        p_name,
        p_value,
        p_needs_restart,
        PropertyHint::RANGE,
        p_hint_string,
    );
}
#[derive(Debug)]
pub struct RapierProjectSettings;
impl RapierProjectSettings {
    pub fn register_settings() {
        let integration_parameters = IntegrationParameters::default();
        register_setting_ranged(
            SOLVER_NUM_INTERNAL_PGS_ITERATIONS,
            Variant::from(DEFAULT_PGS_ITERATIONS),
            "1,8,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS,
            Variant::from(DEFAULT_STABILIZATION_ITERATIONS),
            "1,8,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_ITERATIONS,
            Variant::from(integration_parameters.num_solver_iterations as i32),
            "1,16,or_greater",
            false,
        );
        register_setting_ranged(
            MOTION_RECOVER_ATTEMPTS,
            Variant::from(MOTION_RECOVER_ATTEMPTS_DEFAULT),
            "1,16,1,or_greater",
            false,
        );
        register_setting_ranged(
            MOTION_RECOVER_RATIO,
            Variant::from(MOTION_RECOVER_RATIO_DEFAULT as real),
            "0.05,1,0.05",
            false,
        );
        register_setting_ranged(
            MOTION_CAST_ITERATIONS,
            Variant::from(MOTION_CAST_ITERATIONS_DEFAULT),
            "1,32,1,or_greater",
            false,
        );
        register_setting_ranged(
            MOTION_STUCK_PENETRATION,
            Variant::from(MOTION_STUCK_PENETRATION_DEFAULT as real),
            "0.001,10,0.001,or_greater",
            false,
        );
        register_setting_ranged(
            QUERY_MAX_SHAPE_CAST_RESULTS,
            Variant::from(MAX_SHAPE_CAST_RESULTS_DEFAULT as i64),
            "1,256,1,or_greater",
            false,
        );
        register_setting(
            QUERY_POINT_HONORS_PICKABLE,
            Variant::from(true),
            false,
            PropertyHint::NONE,
            "",
        );
        register_setting_ranged(
            SHAPE_SCALE_SUBDIVISIONS,
            Variant::from(SHAPE_SCALE_SUBDIVISIONS_DEFAULT as i64),
            "4,64,1,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_MAX_CCD_SUBSTEPS,
            Variant::from(DEFAULT_MAX_CCD_SUBSTEPS),
            "0,16,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NORMALIZED_ALLOWED_LINEAR_ERROR,
            Variant::from(integration_parameters.normalized_allowed_linear_error),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NORMALIZED_MAX_CORRECTIVE_VELOCITY,
            Variant::from(integration_parameters.normalized_max_corrective_velocity),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NORMALIZED_PREDICTION_DISTANCE,
            Variant::from(integration_parameters.normalized_prediction_distance),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NORMALIZED_MAX_LINEAR_VELOCITY,
            Variant::from(integration_parameters.normalized_max_linear_velocity),
            "1,10000,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_PREDICTIVE_CONTACT_ALLOWANCE_THRESHOLD,
            Variant::from(integration_parameters.normalized_prediction_distance),
            "0,1,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            CONTACT_DAMPING_RATIO,
            Variant::from(integration_parameters.contact_softness.damping_ratio),
            "0,100,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            CONTACT_NATURAL_FREQUENCY,
            Variant::from(integration_parameters.contact_softness.natural_frequency),
            "0,100,0.00001,or_greater",
            false,
        );
        #[cfg(feature = "dim2")]
        register_setting(
            ORIENTED_CONCAVE_POLYLINE,
            Variant::from(false),
            false,
            PropertyHint::NONE,
            "",
        );
        register_setting_ranged(
            FLUID_PARTICLE_RADIUS,
            Variant::from(FLUID_PARTICLE_VALUE),
            "0.00001,100,0.00001,or_greater",
            true,
        );
        register_setting_ranged(
            FLUID_SMOOTHING_FACTOR,
            Variant::from(2.0),
            "0.00001,10,0.00001,suffix:%,or_greater",
            false,
        );
        register_setting_ranged(
            FLUID_BOUNDARY_COEFF,
            Variant::from(0.00001),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            LENGTH_UNIT,
            Variant::from(LENGTH_UNIT_VALUE),
            LENGTH_UNIT_HINT,
            false,
        );
    }

    fn get_setting_int(p_setting: &str) -> i64 {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting);
        setting_value.to::<i64>()
    }

    fn get_setting_double(p_setting: &str) -> f64 {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting);
        // A value written as an integer -- from override.cfg, or `set_setting(name, 0)` --
        // arrives as an Int variant that a strict float conversion does not survive. Callers
        // use these as thresholds, where a garbage read silently disables the behaviour the
        // setting gates, so fall back through int and never hand back a non-finite number.
        let value = setting_value
            .try_to::<f64>()
            .or_else(|_| setting_value.try_to::<i64>().map(|i| i as f64))
            .unwrap_or(0.0);
        if value.is_finite() { value } else { 0.0 }
    }

    pub fn get_solver_max_ccd_substeps() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_MAX_CCD_SUBSTEPS).max(0)
    }

    pub fn get_solver_num_solver_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_ITERATIONS).max(1)
    }

    pub fn get_solver_num_internal_pgs_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_INTERNAL_PGS_ITERATIONS).max(1)
    }

    pub fn get_fluid_particle_radius() -> Real {
        (RapierProjectSettings::get_setting_double(FLUID_PARTICLE_RADIUS) as Real)
            .max(Real::EPSILON)
    }

    pub fn get_fluid_smoothing_factor() -> Real {
        (RapierProjectSettings::get_setting_double(FLUID_SMOOTHING_FACTOR) as Real)
            .max(Real::EPSILON)
    }

    pub fn get_fluid_boundary_coef() -> Real {
        RapierProjectSettings::get_setting_double(FLUID_BOUNDARY_COEFF) as Real
    }

    pub fn get_length_unit() -> Real {
        (RapierProjectSettings::get_setting_double(LENGTH_UNIT) as Real).max(Real::EPSILON)
    }

    pub fn get_normalized_allowed_linear_error() -> Real {
        RapierProjectSettings::get_setting_double(SOLVER_NORMALIZED_ALLOWED_LINEAR_ERROR) as Real
    }

    pub fn get_normalized_max_corrective_velocity() -> Real {
        RapierProjectSettings::get_setting_double(SOLVER_NORMALIZED_MAX_CORRECTIVE_VELOCITY) as Real
    }

    pub fn get_normalized_prediction_distance() -> Real {
        RapierProjectSettings::get_setting_double(SOLVER_NORMALIZED_PREDICTION_DISTANCE) as Real
    }

    pub fn get_normalized_max_linear_velocity() -> Real {
        RapierProjectSettings::get_setting_double(SOLVER_NORMALIZED_MAX_LINEAR_VELOCITY) as Real
    }

    pub fn get_predictive_contact_allowance_threshold() -> Real {
        RapierProjectSettings::get_setting_double(SOLVER_PREDICTIVE_CONTACT_ALLOWANCE_THRESHOLD)
            as Real
    }

    pub fn get_num_internal_stabilization_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS).max(1)
    }

    pub fn get_contact_damping_ratio() -> Real {
        RapierProjectSettings::get_setting_double(CONTACT_DAMPING_RATIO) as Real
    }

    pub fn get_contact_natural_frequency() -> Real {
        RapierProjectSettings::get_setting_double(CONTACT_NATURAL_FREQUENCY) as Real
    }

    #[cfg(feature = "dim2")]
    pub fn get_oriented_concave_polyline() -> bool {
        let project_settings = ProjectSettings::singleton();
        project_settings
            .get_setting_with_override(ORIENTED_CONCAVE_POLYLINE)
            .to::<bool>()
    }

    /// Whether Godot runs the physics server on its own thread. Read once: Godot decides this
    /// at startup, when it either wraps the server in a command queue or calls it inline.
    pub fn is_run_on_separate_thread() -> bool {
        static ON_SEPARATE_THREAD: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        *ON_SEPARATE_THREAD.get_or_init(|| {
            ProjectSettings::singleton()
                .get_setting_with_override(RUN_ON_SEPARATE_THREAD)
                .try_to()
                .unwrap_or(false)
        })
    }

    /// Deliberately not a project setting: a thread count saved on one machine is wrong on
    /// the next. Computed once per run.
    ///
    /// `GODOT_RAPIER_NUM_THREADS` overrides it for the run. Not a shipping knob -- it exists so
    /// the same scene can be measured at 1 vs N workers, which is the only way to tell a serial
    /// bottleneck apart from a threading-overhead one.
    #[cfg(feature = "parallel")]
    pub fn get_num_threads() -> usize {
        static NUM_THREADS: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
        *NUM_THREADS.get_or_init(|| {
            std::env::var("GODOT_RAPIER_NUM_THREADS")
                .ok()
                .and_then(|v| v.parse::<usize>().ok())
                .filter(|n| *n > 0)
                .unwrap_or_else(|| performance_core_count().max(1))
        })
    }
}
