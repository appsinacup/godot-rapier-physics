use godot::{
    builtin::{meta::ToGodot, Dictionary, Variant, Vector2},
    engine::{global::PropertyHint, ProjectSettings},
};

const SOLVER_NUM_ITERATIONS: &str = "physics/rapier_2d/solver/num_iterations";
const SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS: &str =
    "physics/rapier_2d/solver/num_additional_friction_iterations";
const SOLVER_NUM_INTERNAL_PGS_ITERATIONS: &str =
    "physics/rapier_2d/solver/num_internal_pgs_iterations";
const SOLVER_MAX_CCD_SUBSTEPS: &str = "physics/rapier_2d/solver/max_ccd_substeps";
const FLUID_GRAVITY_DIR: &str = "physics/rapier_2d/fluid/fluid_gravity_dir";
const FLUID_GRAVITY_VALUE: &str = "physics/rapier_2d/fluid/fluid_gravity_value";
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier_2d/fluid/fluid_particle_radius";
const FLUID_SMOOTHING_FACTOR: &str = "physics/rapier_2d/fluid/fluid_smoothing_factor";

fn register_setting(
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
    property_info.insert("name", p_name);
    property_info.insert("type", p_value.get_type());
    property_info.insert("hint", p_hint);
    property_info.insert("hint_string", p_hint_string);
    project_settings.add_property_info(property_info);
    project_settings.set_initial_value(p_name.into(), p_value);
    project_settings.set_restart_if_changed(p_name.into(), p_needs_restart);

    static mut ORDER: i32 = 1000000;
    unsafe {
        project_settings.set_order(p_name.into(), ORDER);
        ORDER += 1;
    }
}

fn register_setting_plain(p_name: &str, p_value: Variant, p_needs_restart: bool) {
    register_setting(p_name, p_value, p_needs_restart, PropertyHint::NONE, "");
}

fn register_setting_ranged(
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
        register_setting_ranged(
            SOLVER_NUM_INTERNAL_PGS_ITERATIONS,
            Variant::from(1),
            "1,4,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS,
            Variant::from(4),
            "1,16,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_NUM_ITERATIONS,
            Variant::from(4),
            "1,16,or_greater",
            false,
        );
        register_setting_ranged(
            SOLVER_MAX_CCD_SUBSTEPS,
            Variant::from(1),
            "1,16,or_greater",
            false,
        );
        register_setting_plain(
            FLUID_GRAVITY_DIR,
            Variant::from(Vector2::new(0.0, 1.0)),
            false,
        );
        register_setting_plain(FLUID_GRAVITY_VALUE, Variant::from(980.0), false);
        register_setting_ranged(
            FLUID_PARTICLE_RADIUS,
            Variant::from(20.0),
            "0,100,0.00001",
            true,
        );
        register_setting_ranged(
            FLUID_SMOOTHING_FACTOR,
            Variant::from(2.0),
            "0,10,0.00001,suffix:%",
            true,
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
    fn get_setting_vector2(p_setting: &str) -> Vector2 {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting.into());
        setting_value.to::<Vector2>()
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

    pub fn get_fluid_gravity_dir() -> Vector2 {
        RapierProjectSettings::get_setting_vector2(FLUID_GRAVITY_DIR)
    }

    pub fn get_fluid_gravity_value() -> f64 {
        RapierProjectSettings::get_setting_double(FLUID_GRAVITY_VALUE)
    }

    pub fn get_fluid_particle_radius() -> f64 {
        RapierProjectSettings::get_setting_double(FLUID_PARTICLE_RADIUS)
    }

    pub fn get_fluid_smoothing_factor() -> f64 {
        RapierProjectSettings::get_setting_double(FLUID_SMOOTHING_FACTOR)
    }
}
