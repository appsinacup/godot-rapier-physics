#include "rapier_project_settings.h"

#include <godot_cpp/classes/project_settings.hpp>

using namespace godot;

constexpr char RUN_ON_SEPARATE_THREAD[] = "physics/2d/run_on_separate_thread";
constexpr char MAX_THREADS[] = "threading/worker_pool/max_threads";
constexpr char SOLVER_ERP[] = "physics/rapier_2d/solver/erp";
constexpr char SOLVER_DAMPING_RATIO[] = "physics/rapier_2d/solver/damping_ratio";
constexpr char SOLVER_JOINT_ERP[] = "physics/rapier_2d/solver/joint_erp";
constexpr char SOLVER_JOINT_DAMPING_RATIO[] = "physics/rapier_2d/solver/joint_damping_ratio";
constexpr char SOLVER_ALLOWED_LINEAR_ERROR[] = "physics/rapier_2d/solver/allowed_linear_error";
constexpr char SOLVER_PREDICTION_DISTANCE[] = "physics/rapier_2d/solver/prediction_distance";
constexpr char SOLVER_NUM_ITERATIONS[] = "physics/rapier_2d/solver/num_iterations";
constexpr char SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS[] = "physics/rapier_2d/solver/num_additional_friction_iterations";
constexpr char SOLVER_NUM_INTERNAL_PGS_ITERATIONS[] = "physics/rapier_2d/solver/num_internal_pgs_iterations";
constexpr char SOLVER_MAX_CCD_SUBSTEPS[] = "physics/rapier_2d/solver/max_ccd_substeps";
constexpr char FLUID_GRAVITY_DIR[] = "physics/rapier_2d/fluid/fluid_gravity_dir";
constexpr char FLUID_GRAVITY_VALUE[] = "physics/rapier_2d/fluid/fluid_gravity_value";
constexpr char FLUID_PARTICLE_RADIUS[] = "physics/rapier_2d/fluid/fluid_particle_radius";
constexpr char FLUID_SMOOTHING_FACTOR[] = "physics/rapier_2d/fluid/fluid_smoothing_factor";
constexpr char FLUID_DRAW_DEBUG[] = "physics/rapier_2d/fluid/fluid_draw_debug";

void register_setting(
		const String &p_name,
		const Variant &p_value,
		bool p_needs_restart,
		PropertyHint p_hint,
		const String &p_hint_string) {
	ProjectSettings *project_settings = ProjectSettings::get_singleton();

	if (!project_settings->has_setting(p_name)) {
		project_settings->set(p_name, p_value);
	}

	Dictionary property_info;
	property_info["name"] = p_name;
	property_info["type"] = p_value.get_type();
	property_info["hint"] = p_hint;
	property_info["hint_string"] = p_hint_string;

	project_settings->add_property_info(property_info);
	project_settings->set_initial_value(p_name, p_value);
	project_settings->set_restart_if_changed(p_name, p_needs_restart);

	// HACK(mihe): We want our settings to appear in the order we register them in, but if we start
	// the order at 0 we end up moving the entire `physics/` group to the top of the tree view, so
	// instead we give it a hefty starting order and increment from there, which seems to give us
	// the desired effect.
	static int32_t order = 1000000;

	project_settings->set_order(p_name, order++);
}

void register_setting_plain(
		const String &p_name,
		const Variant &p_value,
		bool p_needs_restart = false) {
	register_setting(p_name, p_value, p_needs_restart, PROPERTY_HINT_NONE, {});
}

void register_setting_hinted(
		const String &p_name,
		const Variant &p_value,
		const String &p_hint_string,
		bool p_needs_restart = false) {
	register_setting(p_name, p_value, p_needs_restart, PROPERTY_HINT_NONE, p_hint_string);
}

void register_setting_ranged(
		const String &p_name,
		const Variant &p_value,
		const String &p_hint_string,
		bool p_needs_restart = false) {
	register_setting(p_name, p_value, p_needs_restart, PROPERTY_HINT_RANGE, p_hint_string);
}

void RapierProjectSettings::register_settings() {
	register_setting_ranged(SOLVER_ERP, 0.6, U"0.00001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_DAMPING_RATIO, 1.0, U"0.00001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_JOINT_ERP, 1.0, U"0.00001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_JOINT_DAMPING_RATIO, 1.0, U"0.0001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_ALLOWED_LINEAR_ERROR, 0.001, U"0,1,0.00001,suffix:m");
	register_setting_ranged(SOLVER_PREDICTION_DISTANCE, 0.002, U"0,1,0.00001,suffix:m");
	register_setting_ranged(SOLVER_NUM_INTERNAL_PGS_ITERATIONS, 1, U"1,4,or_greater");
	register_setting_ranged(SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS, 4, U"1,16,or_greater");
	register_setting_ranged(SOLVER_NUM_ITERATIONS, 4, U"1,16,or_greater");
	register_setting_ranged(SOLVER_MAX_CCD_SUBSTEPS, 1, U"1,16,or_greater");
	register_setting_plain(FLUID_GRAVITY_DIR, Vector2(0.0, 1.0), "");
	register_setting_plain(FLUID_GRAVITY_VALUE, 980.0, "");
	register_setting_ranged(FLUID_PARTICLE_RADIUS, 10.0, U"0,100,0.00001,suffix:m");
	register_setting_ranged(FLUID_SMOOTHING_FACTOR, 2.0, U"0,10,0.00001,suffix:%");
	register_setting_plain(FLUID_DRAW_DEBUG, false, "");
}

template <typename TType>
TType get_setting(const char *p_setting) {
	const ProjectSettings *project_settings = ProjectSettings::get_singleton();
	const Variant setting_value = project_settings->get_setting_with_override(p_setting);
	const Variant::Type setting_type = setting_value.get_type();
	const Variant::Type expected_type = Variant(TType()).get_type();

	ERR_FAIL_COND_V(setting_type != expected_type, Variant());

	return setting_value;
}

bool RapierProjectSettings::should_run_on_separate_thread() {
	return get_setting<bool>(RUN_ON_SEPARATE_THREAD);
}

int RapierProjectSettings::get_max_threads() {
	return get_setting<int>(MAX_THREADS);
}
double RapierProjectSettings::get_solver_erp() {
	return get_setting<double>(SOLVER_ERP);
}
double RapierProjectSettings::get_solver_damping_ratio() {
	return get_setting<double>(SOLVER_DAMPING_RATIO);
}
double RapierProjectSettings::get_solver_joint_erp() {
	return get_setting<double>(SOLVER_JOINT_ERP);
}
double RapierProjectSettings::get_solver_joint_damping_ratio() {
	return get_setting<double>(SOLVER_JOINT_DAMPING_RATIO);
}
double RapierProjectSettings::get_solver_allowed_linear_error() {
	return get_setting<double>(SOLVER_ALLOWED_LINEAR_ERROR);
}
int RapierProjectSettings::get_solver_max_ccd_substeps() {
	return get_setting<int>(SOLVER_MAX_CCD_SUBSTEPS);
}
double RapierProjectSettings::get_solver_prediction_distance() {
	return get_setting<double>(SOLVER_PREDICTION_DISTANCE);
}
int RapierProjectSettings::get_solver_num_solver_iterations() {
	return get_setting<int>(SOLVER_NUM_ITERATIONS);
}
int RapierProjectSettings::get_solver_num_additional_friction_iterations() {
	return get_setting<int>(SOLVER_NUM_ADDITIONAL_FRICTION_ITERATIONS);
}
int RapierProjectSettings::get_solver_num_internal_pgs_iterations() {
	return get_setting<int>(SOLVER_NUM_INTERNAL_PGS_ITERATIONS);
}

Vector2 RapierProjectSettings::get_fluid_gravity_dir() {
	return get_setting<Vector2>(FLUID_GRAVITY_DIR);
}

double RapierProjectSettings::get_fluid_gravity_value() {
	return get_setting<double>(FLUID_GRAVITY_VALUE);
}

double RapierProjectSettings::get_fluid_particle_radius() {
	return get_setting<double>(FLUID_PARTICLE_RADIUS);
}

double RapierProjectSettings::get_fluid_smoothing_factor() {
	return get_setting<double>(FLUID_SMOOTHING_FACTOR);
}

bool RapierProjectSettings::get_fluid_draw_debug() {
	return get_setting<bool>(FLUID_DRAW_DEBUG);
}