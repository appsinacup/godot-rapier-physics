use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use godot::classes::*;
use godot::global::*;
use godot::prelude::*;
use rapier::dynamics::IntegrationParameters;
use rapier::math::Real;
#[cfg(feature = "parallel")]
const NUM_THREADS: &str = "physics/rapier/parallel/num_threads";
const SOLVER_PRESET: &str = "physics/rapier/solver/preset";
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
const SOLVER_PREDICTIVE_CONTACT_ALLOWANCE_THRESHOLD: &str =
    "physics/rapier/solver/predictive_contact_allowance_threshold";
const CONTACT_DAMPING_RATIO: &str = "physics/rapier/solver/contact_damping_ratio";
const CONTACT_NATURAL_FREQUENCY: &str = "physics/rapier/solver/contact_natural_frequency";
// Stability preset constants
const STABILITY_PGS_ITERATIONS: i64 = 4;
const STABILITY_STABILIZATION_ITERATIONS: i64 = 4;
const STABILITY_DAMPING_RATIO: f64 = 20.0;
const STABILITY_NATURAL_FREQUENCY: f64 = 50.0;
#[cfg(feature = "dim2")]
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier/fluid/fluid_particle_radius_2d";
#[cfg(feature = "dim3")]
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier/fluid/fluid_particle_radius_3d";
const FLUID_SMOOTHING_FACTOR: &str = "physics/rapier/fluid/fluid_smoothing_factor";
const FLUID_BOUNDARY_COEFF: &str = "physics/rapier/fluid/fluid_boundary_coefficient";
#[cfg(feature = "dim2")]
const LENGTH_UNIT: &str = "physics/rapier/solver/length_unit_2d";
#[cfg(feature = "dim2")]
const GHOST_COLLISION_DISTANCE: &str = "physics/rapier/logic/ghost_collision_distance_2d";
#[cfg(feature = "dim3")]
const GHOST_COLLISION_DISTANCE: &str = "physics/rapier/logic/ghost_collision_distance_3d";
#[cfg(feature = "dim2")]
const GHOST_COLLISION_DISTANCE_DEFAULT: real = 0.1;
#[cfg(feature = "dim3")]
const GHOST_COLLISION_DISTANCE_DEFAULT: real = 0.001;
#[cfg(feature = "dim2")]
const LENGTH_UNIT_VALUE: real = 100.0;
#[cfg(feature = "dim2")]
const FLUID_PARTICLE_VALUE: real = 20.0;
#[cfg(feature = "dim3")]
const FLUID_PARTICLE_VALUE: real = 0.5;
#[cfg(feature = "dim3")]
const LENGTH_UNIT: &str = "physics/rapier/solver/length_unit_3d";
#[cfg(feature = "dim3")]
const LENGTH_UNIT_VALUE: real = 1.0;
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
#[derive(Debug, Clone, Copy)]
pub enum RapierSolverPreset {
    Performance = 0,
    Stability = 1,
    Custom = 2,
}
impl RapierSolverPreset {
    pub fn from_i64(value: i64) -> Self {
        match value {
            1 => RapierSolverPreset::Stability,
            2 => RapierSolverPreset::Custom,
            _ => RapierSolverPreset::Performance,
        }
    }
}
#[derive(Debug)]
pub struct RapierProjectSettings;
impl RapierProjectSettings {
    pub fn register_settings() {
        let integration_parameters = IntegrationParameters::default();
        #[cfg(feature = "parallel")]
        {
            let num_threads = num_cpus::get_physical();
            register_setting_ranged(
                NUM_THREADS,
                Variant::from(num_threads as i32),
                "1,64,or_greater",
                false,
            );
        }
        // Register preset setting first
        register_setting(
            SOLVER_PRESET,
            Variant::from(0i32),
            false,
            PropertyHint::ENUM,
            "Performance,Stability,Custom",
        );
        register_setting_ranged(
            SOLVER_NUM_INTERNAL_PGS_ITERATIONS,
            Variant::from(integration_parameters.num_internal_pgs_iterations as i32),
            "1,8,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS,
            Variant::from(integration_parameters.num_internal_stabilization_iterations as i32),
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
            SOLVER_MAX_CCD_SUBSTEPS,
            Variant::from(integration_parameters.max_ccd_substeps as i32),
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
        register_setting_ranged(
            GHOST_COLLISION_DISTANCE,
            Variant::from(GHOST_COLLISION_DISTANCE_DEFAULT),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            FLUID_PARTICLE_RADIUS,
            Variant::from(FLUID_PARTICLE_VALUE),
            "0,100,0.00001,or_greater",
            true,
        );
        register_setting_ranged(
            FLUID_SMOOTHING_FACTOR,
            Variant::from(2.0),
            "0,10,0.00001,suffix:%,or_greater",
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
            "1,100,1,suffix:length_unit,or_greater",
            false,
        );
        static SIGNAL_CONNECTED: AtomicBool = AtomicBool::new(false);
        if !SIGNAL_CONNECTED.swap(true, Ordering::AcqRel) {
            let project_settings = ProjectSettings::singleton();
            let mut signals = project_settings.signals();
            signals.settings_changed().connect(|| {
                static LAST_PRESET_VALUE: std::sync::Mutex<Option<i64>> =
                    std::sync::Mutex::new(None);
                static APPLYING_PRESET: AtomicBool = AtomicBool::new(false);
                let current_preset = RapierProjectSettings::get_setting_int(SOLVER_PRESET);
                let mut last_preset_value = LAST_PRESET_VALUE.lock().unwrap();
                if let Some(last) = *last_preset_value {
                    if last != current_preset {
                        // Preset setting changed - apply the new preset
                        if !APPLYING_PRESET.load(Ordering::Acquire) {
                            RapierProjectSettings::apply_preset();
                        }
                        *last_preset_value = Some(current_preset);
                        return;
                    }
                } else {
                    // First time - just store the current value
                    *last_preset_value = Some(current_preset);
                    return;
                }
                // If we're currently applying a preset, don't detect changes
                if APPLYING_PRESET.load(Ordering::Acquire) {
                    return;
                }
                // Check if current preset is already Custom - if so, don't change it
                if current_preset == RapierSolverPreset::Custom as i64 {
                    return;
                }
                // Individual settings changed - detect if we should switch to Custom
                let detected = RapierProjectSettings::detect_current_preset();
                if matches!(detected, RapierSolverPreset::Custom) {
                    // Settings no longer match any preset - switch to Custom
                    let mut project_settings = ProjectSettings::singleton();
                    project_settings.set(
                        SOLVER_PRESET,
                        &Variant::from(RapierSolverPreset::Custom as i32),
                    );
                    *last_preset_value = Some(RapierSolverPreset::Custom as i64);
                }
            });
        }
    }

    fn get_setting_int(p_setting: &str) -> i64 {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting);
        setting_value.to::<i64>()
    }

    fn get_setting_double(p_setting: &str) -> f64 {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting);
        setting_value.to::<f64>()
    }

    pub fn get_solver_preset() -> RapierSolverPreset {
        RapierSolverPreset::from_i64(RapierProjectSettings::get_setting_int(SOLVER_PRESET))
    }

    pub fn detect_current_preset() -> RapierSolverPreset {
        let pgs = RapierProjectSettings::get_setting_int(SOLVER_NUM_INTERNAL_PGS_ITERATIONS);
        let stab =
            RapierProjectSettings::get_setting_int(SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS);
        let damping = RapierProjectSettings::get_setting_double(CONTACT_DAMPING_RATIO);
        let freq = RapierProjectSettings::get_setting_double(CONTACT_NATURAL_FREQUENCY);
        // Check if matches Stability preset
        if pgs == STABILITY_PGS_ITERATIONS
            && stab == STABILITY_STABILIZATION_ITERATIONS
            && (damping - STABILITY_DAMPING_RATIO).abs() < 0.001
            && (freq - STABILITY_NATURAL_FREQUENCY).abs() < 0.001
        {
            return RapierSolverPreset::Stability;
        }
        // Check if matches Performance preset (default Rapier values)
        let integration_parameters = IntegrationParameters::default();
        if pgs == integration_parameters.num_internal_pgs_iterations as i64
            && stab == integration_parameters.num_internal_stabilization_iterations as i64
            && (damping - integration_parameters.contact_softness.damping_ratio as f64).abs()
                < 0.001
            && (freq - integration_parameters.contact_softness.natural_frequency as f64).abs()
                < 0.001
        {
            return RapierSolverPreset::Performance;
        }
        RapierSolverPreset::Custom
    }

    pub fn apply_preset() {
        let preset = RapierProjectSettings::get_solver_preset();
        if matches!(preset, RapierSolverPreset::Custom) {
            return; // Don't apply when Custom
        }
        // Set flag to prevent detection during application
        static APPLYING_PRESET: AtomicBool = AtomicBool::new(false);
        APPLYING_PRESET.store(true, Ordering::Release);
        let mut project_settings = ProjectSettings::singleton();
        match preset {
            RapierSolverPreset::Stability => {
                // Stability preset
                project_settings.set(
                    SOLVER_NUM_INTERNAL_PGS_ITERATIONS,
                    &Variant::from(STABILITY_PGS_ITERATIONS as i32),
                );
                project_settings.set(
                    SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS,
                    &Variant::from(STABILITY_STABILIZATION_ITERATIONS as i32),
                );
                project_settings.set(
                    CONTACT_DAMPING_RATIO,
                    &Variant::from(STABILITY_DAMPING_RATIO),
                );
                project_settings.set(
                    CONTACT_NATURAL_FREQUENCY,
                    &Variant::from(STABILITY_NATURAL_FREQUENCY),
                );
            }
            RapierSolverPreset::Performance => {
                // Performance preset (default Rapier values)
                let integration_parameters = IntegrationParameters::default();
                project_settings.set(
                    SOLVER_NUM_INTERNAL_PGS_ITERATIONS,
                    &Variant::from(integration_parameters.num_internal_pgs_iterations as i32),
                );
                project_settings.set(
                    SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS,
                    &Variant::from(
                        integration_parameters.num_internal_stabilization_iterations as i32,
                    ),
                );
                project_settings.set(
                    CONTACT_DAMPING_RATIO,
                    &Variant::from(integration_parameters.contact_softness.damping_ratio),
                );
                project_settings.set(
                    CONTACT_NATURAL_FREQUENCY,
                    &Variant::from(integration_parameters.contact_softness.natural_frequency),
                );
            }
            RapierSolverPreset::Custom => {} // Do nothing for Custom
        }
        APPLYING_PRESET.store(false, Ordering::Release);
    }

    pub fn get_solver_max_ccd_substeps() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_MAX_CCD_SUBSTEPS)
    }

    pub fn get_solver_num_solver_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_ITERATIONS)
    }

    pub fn get_solver_num_internal_pgs_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_INTERNAL_PGS_ITERATIONS)
    }

    pub fn get_fluid_particle_radius() -> Real {
        RapierProjectSettings::get_setting_double(FLUID_PARTICLE_RADIUS) as Real
    }

    pub fn get_fluid_smoothing_factor() -> Real {
        RapierProjectSettings::get_setting_double(FLUID_SMOOTHING_FACTOR) as Real
    }

    pub fn get_fluid_boundary_coef() -> Real {
        RapierProjectSettings::get_setting_double(FLUID_BOUNDARY_COEFF) as Real
    }

    pub fn get_length_unit() -> Real {
        RapierProjectSettings::get_setting_double(LENGTH_UNIT) as Real
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

    pub fn get_predictive_contact_allowance_threshold() -> Real {
        RapierProjectSettings::get_setting_double(SOLVER_PREDICTIVE_CONTACT_ALLOWANCE_THRESHOLD)
            as Real
    }

    pub fn get_num_internal_stabilization_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS)
    }

    pub fn get_contact_damping_ratio() -> Real {
        RapierProjectSettings::get_setting_double(CONTACT_DAMPING_RATIO) as Real
    }

    pub fn get_contact_natural_frequency() -> Real {
        RapierProjectSettings::get_setting_double(CONTACT_NATURAL_FREQUENCY) as Real
    }

    pub fn get_ghost_collision_distance() -> Real {
        RapierProjectSettings::get_setting_double(GHOST_COLLISION_DISTANCE) as Real
    }

    #[cfg(feature = "parallel")]
    pub fn get_num_threads() -> usize {
        RapierProjectSettings::get_setting_int(NUM_THREADS) as usize
    }
}
