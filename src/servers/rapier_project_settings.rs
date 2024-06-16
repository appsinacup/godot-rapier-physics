use godot::classes::*;
use godot::global::*;
use godot::prelude::*;
use rapier::math::Real;
const SOLVER_NUM_ITERATIONS: &str = "physics/rapier/solver/num_iterations";
const SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS: &str =
    "physics/rapier/solver/num_additional_friction_iterations";
const SOLVER_NUM_INTERNAL_PGS_ITERATIONS: &str =
    "physics/rapier/solver/num_internal_pgs_iterations";
const SOLVER_MAX_CCD_SUBSTEPS: &str = "physics/rapier/solver/max_ccd_substeps";
const CONTACT_SKIN: &str = "physics/rapier/solver/polygon_contact_skin";
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier/fluid/fluid_particle_radius";
const FLUID_SMOOTHING_FACTOR: &str = "physics/rapier/fluid/fluid_smoothing_factor";
pub const MONITORS: [&str; 4] = ["inner_step", "step", "before_step", "flush"];
#[cfg(feature = "dim2")]
const LENGTH_UNIT: &str = "physics/rapier/solver/length_unit_2d";
#[cfg(feature = "dim2")]
const LENGTH_UNIT_VALUE: real = 100.0;
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
pub fn register_setting_plain(p_name: &str, p_value: Variant, p_needs_restart: bool) {
    register_setting(p_name, p_value, p_needs_restart, PropertyHint::NONE, "");
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
        register_setting_ranged(
            SOLVER_NUM_INTERNAL_PGS_ITERATIONS,
            Variant::from(1),
            "1,4,or_greater",
            true,
        );
        register_setting_ranged(
            SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS,
            Variant::from(4),
            "1,16,or_greater",
            true,
        );
        register_setting_ranged(
            SOLVER_NUM_ITERATIONS,
            Variant::from(4),
            "1,16,or_greater",
            true,
        );
        register_setting_ranged(
            SOLVER_MAX_CCD_SUBSTEPS,
            Variant::from(1),
            "1,16,or_greater",
            true,
        );
        register_setting_ranged(
            CONTACT_SKIN,
            Variant::from(0.0),
            "0,10,0.00001,or_greater",
            true,
        );
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
        register_setting_ranged(
            LENGTH_UNIT,
            Variant::from(LENGTH_UNIT_VALUE),
            "1,100,1,suffix:length_unit",
            true,
        );
        for monitor in MONITORS {
            register_setting_plain(
                ("physics/rapier/monitor/".to_string() + monitor).as_str(),
                Variant::from(false),
                true,
            );
        }
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

    fn get_setting_bool(p_setting: &str) -> bool {
        let project_settings = ProjectSettings::singleton();
        let setting_value = project_settings.get_setting_with_override(p_setting.into());
        setting_value.to::<bool>()
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

    pub fn get_monitor_enabled(monitor: &str) -> bool {
        RapierProjectSettings::get_setting_bool(
            ("physics/rapier/monitor/".to_string() + monitor).as_str(),
        )
    }

    pub fn counters_enabled() -> bool {
        for monitor in MONITORS {
            if RapierProjectSettings::get_monitor_enabled(monitor) {
                return true;
            }
        }
        false
    }
}
