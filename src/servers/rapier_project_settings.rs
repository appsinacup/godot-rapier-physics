use godot::classes::*;
use godot::global::*;
use godot::prelude::*;
use rapier::dynamics::IntegrationParameters;
use rapier::math::Real;
const SOLVER_NUM_ITERATIONS: &str = "physics/rapier/solver/num_iterations";
const SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS: &str =
    "physics/rapier/solver/num_internal_stabilization_iterations";
const SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS: &str =
    "physics/rapier/solver/num_additional_friction_iterations";
const SOLVER_NUM_INTERNAL_PGS_ITERATIONS: &str =
    "physics/rapier/solver/num_internal_pgs_iterations";
const SOLVER_MAX_CCD_SUBSTEPS: &str = "physics/rapier/solver/max_ccd_substeps";
const SOLVER_NORMALIZED_ALLOWED_LINEAR_ERROR: &str =
    "physics/rapier/solver/normalized_allowed_linear_error";
const SOLVER_NORMALIZED_MAX_CORRECTIVE_VELOCITY: &str =
    "physics/rapier/solver/normalized_max_corrective_velocity";
const SOLVER_NORMALIZED_PREDICTION_DISTANCE: &str =
    "physics/rapier/solver/normalized_prediction_distance";
const CONTACT_SKIN: &str = "physics/rapier/solver/contact_skin";
const CONTACT_DAMPING_RATIO: &str = "physics/rapier/solver/contact_damping_ratio";
const CONTACT_NATURAL_FREQUENCY: &str = "physics/rapier/solver/contact_natural_frequency";
#[cfg(feature = "dim2")]
const GHOST_COLLISION_DISTANCE: &str = "physics/rapier/logic/ghost_collision_distance_2d";
#[cfg(feature = "dim3")]
const GHOST_COLLISION_DISTANCE: &str = "physics/rapier/logic/ghost_collision_distance_3d";
#[cfg(feature = "dim2")]
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier/fluid/fluid_particle_radius_2d";
#[cfg(feature = "dim3")]
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier/fluid/fluid_particle_radius_3d";
const FLUID_SMOOTHING_FACTOR: &str = "physics/rapier/fluid/fluid_smoothing_factor";
#[cfg(feature = "dim2")]
const LENGTH_UNIT: &str = "physics/rapier/solver/length_unit_2d";
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
const JOINT_DAMPING_RATIO: &str = "physics/rapier/joint/damping_ratio";
const JOINT_NATURAL_FREQUENCY: &str = "physics/rapier/joint/natural_frequency";
pub fn register_setting(
    p_name: &str,
    p_value: Variant,
    p_needs_restart: bool,
    p_hint: PropertyHint,
    p_hint_string: &str,
) {
    let mut project_settings = ProjectSettings::singleton();
    if !project_settings.has_setting(p_name.into_godot()) {
        project_settings.set(p_name.into(), p_value.clone());
    }
    let mut property_info = Dictionary::new();
    let _ = property_info.insert("name", p_name);
    let _ = property_info.insert("type", p_value.get_type());
    let _ = property_info.insert("hint", p_hint);
    let _ = property_info.insert("hint_string", p_hint_string);
    project_settings.add_property_info(property_info);
    project_settings.set_initial_value(p_name.into(), p_value);
    project_settings.set_restart_if_changed(p_name.into(), p_needs_restart);
    static mut ORDER: i32 = 1000000;
    unsafe {
        project_settings.set_order(p_name.into(), ORDER);
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
            Variant::from(integration_parameters.num_internal_pgs_iterations as i32),
            "1,4,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_INTERNAL_STABILIZATION_ITERATIONS,
            Variant::from(integration_parameters.num_internal_stabilization_iterations as i32),
            "1,4,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS,
            Variant::from(integration_parameters.num_additional_friction_iterations as i32),
            "0,16,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_ITERATIONS,
            Variant::from(integration_parameters.num_solver_iterations.get() as i32),
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
            CONTACT_DAMPING_RATIO,
            Variant::from(integration_parameters.contact_damping_ratio),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            CONTACT_NATURAL_FREQUENCY,
            Variant::from(integration_parameters.contact_natural_frequency),
            "0,100,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            CONTACT_SKIN,
            Variant::from(0.0),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            JOINT_DAMPING_RATIO,
            Variant::from(integration_parameters.joint_damping_ratio),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            JOINT_NATURAL_FREQUENCY,
            Variant::from(integration_parameters.joint_natural_frequency),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            GHOST_COLLISION_DISTANCE,
            Variant::from(0.0),
            "0,10,0.00001,or_greater",
            false,
        );
        register_setting_ranged(
            FLUID_PARTICLE_RADIUS,
            Variant::from(FLUID_PARTICLE_VALUE),
            "0,100,0.00001",
            true,
        );
        register_setting_ranged(
            FLUID_SMOOTHING_FACTOR,
            Variant::from(2.0),
            "0,10,0.00001,suffix:%",
            false,
        );
        register_setting_ranged(
            LENGTH_UNIT,
            Variant::from(LENGTH_UNIT_VALUE),
            "1,100,1,suffix:length_unit",
            false,
        );
    }

    fn get_setting_int(p_setting: &str) -> i64 {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting.into());
        setting_value.to::<i64>()
    }

    fn get_setting_double(p_setting: &str) -> f64 {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting.into());
        setting_value.to::<f64>()
    }

    pub fn get_solver_max_ccd_substeps() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_MAX_CCD_SUBSTEPS)
    }

    pub fn get_solver_num_solver_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_ITERATIONS)
    }

    pub fn get_solver_num_additional_friction_iterations() -> i64 {
        RapierProjectSettings::get_setting_int(SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS)
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

    pub fn get_contact_skin() -> Real {
        RapierProjectSettings::get_setting_double(CONTACT_SKIN) as Real
    }

    pub fn get_length_unit() -> Real {
        RapierProjectSettings::get_setting_double(LENGTH_UNIT) as Real
    }

    pub fn get_joint_damping_ratio() -> Real {
        RapierProjectSettings::get_setting_double(JOINT_DAMPING_RATIO) as Real
    }

    pub fn get_joint_natural_frequency() -> Real {
        RapierProjectSettings::get_setting_double(JOINT_NATURAL_FREQUENCY) as Real
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
}
