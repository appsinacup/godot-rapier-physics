use godot::{
    builtin::{meta::ToGodot, Dictionary, Variant, Vector2},
    engine::{global::PropertyHint, ProjectSettings},
};

use crate::servers2d::rapier_project_settings_2d::{
    register_setting_plain, register_setting_ranged,
};

const SOLVER_NUM_ITERATIONS: &str = "physics/rapier_3d/solver/num_iterations";
const SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS: &str =
    "physics/rapier_3d/solver/num_additional_friction_iterations";
const SOLVER_NUM_INTERNAL_PGS_ITERATIONS: &str =
    "physics/rapier_3d/solver/num_internal_pgs_iterations";
const SOLVER_MAX_CCD_SUBSTEPS: &str = "physics/rapier_3d/solver/max_ccd_substeps";
const CONTACT_SKIN: &str = "physics/rapier_3d/solver/polygon_contact_skin";
const FLUID_GRAVITY_DIR: &str = "physics/rapier_3d/fluid/fluid_gravity_dir";
const FLUID_GRAVITY_VALUE: &str = "physics/rapier_3d/fluid/fluid_gravity_value";
const FLUID_PARTICLE_RADIUS: &str = "physics/rapier_3d/fluid/fluid_particle_radius";
const FLUID_SMOOTHING_FACTOR: &str = "physics/rapier_3d/fluid/fluid_smoothing_factor";

#[derive(Debug)]
pub struct RapierProjectSettings3D;

impl RapierProjectSettings3D {
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
        register_setting_ranged(
            CONTACT_SKIN,
            Variant::from(0.0),
            "0,10,0.00001,or_greater",
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
        RapierProjectSettings3D::get_setting_int(SOLVER_MAX_CCD_SUBSTEPS)
    }

    pub fn get_solver_num_solver_iterations() -> i64 {
        RapierProjectSettings3D::get_setting_int(SOLVER_NUM_ITERATIONS)
    }

    pub fn get_solver_num_additional_friction_iterations() -> i64 {
        RapierProjectSettings3D::get_setting_int(SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS)
    }

    pub fn get_solver_num_internal_pgs_iterations() -> i64 {
        RapierProjectSettings3D::get_setting_int(SOLVER_NUM_INTERNAL_PGS_ITERATIONS)
    }

    pub fn get_fluid_gravity_dir() -> Vector2 {
        RapierProjectSettings3D::get_setting_vector2(FLUID_GRAVITY_DIR)
    }

    pub fn get_fluid_gravity_value() -> f64 {
        RapierProjectSettings3D::get_setting_double(FLUID_GRAVITY_VALUE)
    }

    pub fn get_fluid_particle_radius() -> f64 {
        RapierProjectSettings3D::get_setting_double(FLUID_PARTICLE_RADIUS)
    }

    pub fn get_fluid_smoothing_factor() -> f64 {
        RapierProjectSettings3D::get_setting_double(FLUID_SMOOTHING_FACTOR)
    }

    pub fn get_contact_skin() -> f64 {
        RapierProjectSettings3D::get_setting_double(CONTACT_SKIN)
    }
}
