#include "rapier_project_settings.h"

#include <godot_cpp/classes/project_settings.hpp>

using namespace godot;

constexpr char RUN_ON_SEPARATE_THREAD[] = "physics/2d/run_on_separate_thread";
constexpr char MAX_THREADS[] = "threading/worker_pool/max_threads";
constexpr char SOLVER_MIN_CCD_DT[] = "physics/rapier_2d/solver/min_ccd_dt";
constexpr char SOLVER_ERP[] = "physics/rapier_2d/solver/erp";
constexpr char SOLVER_DAMPING_RATIO[] = "physics/rapier_2d/solver/damping_ratio";
constexpr char SOLVER_JOINT_ERP[] = "physics/rapier_2d/solver/joint_erp";
constexpr char SOLVER_JOINT_DAMPING_RATIO[] = "physics/rapier_2d/solver/joint_damping_ratio";
constexpr char SOLVER_ALLOWED_LINEAR_ERROR[] = "physics/rapier_2d/solver/allowed_linear_error";
constexpr char SOLVER_MAX_PENETRATION_CORRECTION[] = "physics/rapier_2d/solver/max_penetration_correction";
constexpr char SOLVER_PREDICTION_DISTANCE[] = "physics/rapier_2d/solver/prediction_distance";
constexpr char SOLVER_MAX_VELOCITY_ITERATIONS[] = "physics/rapier_2d/solver/max_velocity_iterations";
constexpr char SOLVER_MAX_VELOCITY_FRICTION_ITERATIONS[] = "physics/rapier_2d/solver/max_velocity_friction_iterations";
constexpr char SOLVER_MAX_STABILIZATION_ITERATIONS[] = "physics/rapier_2d/solver/max_stabilization_iterations";
constexpr char SOLVER_INTERLEAVE_RESTITUTION_AND_FRICTION_RESOLUTION[] = "physics/rapier_2d/solver/interleave_restitution_and_friction_resolution";
constexpr char SOLVER_MIN_ISLAND_SIZE[] = "physics/rapier_2d/solver/min_island_size";
constexpr char SOLVER_MAX_CCD_SUBSTEPS[] = "physics/rapier_2d/solver/max_ccd_substeps";

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
	register_setting_ranged(SOLVER_MIN_CCD_DT, 1.0 / 60.0 / 100.0, U"0.0000001,1,0.0000001,suffix:1/s");
	register_setting_ranged(SOLVER_ERP, 0.85, U"0.00001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_DAMPING_RATIO, 0.35, U"0.00001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_JOINT_ERP, 1.0, U"0.00001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_JOINT_DAMPING_RATIO, 0.25, U"0.0001,1,0.00001,suffix:%");
	register_setting_ranged(SOLVER_ALLOWED_LINEAR_ERROR, 0.001, U"0,1,0.00001,suffix:m");
	register_setting_ranged(SOLVER_MAX_PENETRATION_CORRECTION, 3.40282e+38, U"0, 3.40282e+38f,1,suffix:m");
	register_setting_ranged(SOLVER_PREDICTION_DISTANCE, 0.002, U"0,1,0.00001,suffix:m");
	register_setting_ranged(SOLVER_MAX_VELOCITY_ITERATIONS, 19, U"1,16,or_greater");
	register_setting_ranged(SOLVER_MAX_VELOCITY_FRICTION_ITERATIONS, 27, U"1,16,or_greater");
	register_setting_ranged(SOLVER_MAX_STABILIZATION_ITERATIONS, 1, U"1,16,or_greater");
	register_setting_plain(SOLVER_INTERLEAVE_RESTITUTION_AND_FRICTION_RESOLUTION, true);
	register_setting_plain(SOLVER_MIN_ISLAND_SIZE, 128);
	register_setting_plain(SOLVER_MAX_CCD_SUBSTEPS, 1);
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

double RapierProjectSettings::get_solver_min_ccd_dt() {
	return get_setting<double>(SOLVER_MIN_CCD_DT);
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
double RapierProjectSettings::get_solver_max_penetration_correction() {
	return get_setting<double>(SOLVER_MAX_PENETRATION_CORRECTION);
}
double RapierProjectSettings::get_solver_prediction_distance() {
	return get_setting<double>(SOLVER_PREDICTION_DISTANCE);
}
int RapierProjectSettings::get_solver_max_velocity_iterations() {
	return get_setting<int>(SOLVER_MAX_VELOCITY_ITERATIONS);
}
int RapierProjectSettings::get_solver_max_velocity_friction_iterations() {
	return get_setting<int>(SOLVER_MAX_VELOCITY_FRICTION_ITERATIONS);
}
int RapierProjectSettings::get_solver_max_stabilization_iterations() {
	return get_setting<int>(SOLVER_MAX_STABILIZATION_ITERATIONS);
}
bool RapierProjectSettings::get_solver_interleave_restitution_and_friction_resolution() {
	return get_setting<bool>(SOLVER_INTERLEAVE_RESTITUTION_AND_FRICTION_RESOLUTION);
}
int RapierProjectSettings::get_solver_min_island_size() {
	return get_setting<int>(SOLVER_MIN_ISLAND_SIZE);
}
int RapierProjectSettings::get_solver_max_ccd_substeps() {
	return get_setting<int>(SOLVER_MAX_CCD_SUBSTEPS);
}
