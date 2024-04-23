#ifndef RAPIER_PROJECT_SETTINGS_H
#define RAPIER_PROJECT_SETTINGS_H

#include <godot_cpp/variant/vector2.hpp>

using namespace godot;

// See /rapier/src/dynamics/integration_parameters.rs for description of parameters
class RapierProjectSettings {
public:
	static void register_settings();

	static bool should_run_on_separate_thread();
	static int get_max_threads();
	static int get_solver_num_solver_iterations();
	static int get_solver_num_additional_friction_iterations();
	static int get_solver_num_internal_pgs_iterations();
	static int get_solver_max_ccd_substeps();
	static Vector2 get_fluid_gravity_dir();
	static double get_fluid_gravity_value();
	static double get_fluid_smoothing_factor();
	static double get_fluid_particle_radius();
};

#endif // RAPIER_PROJECT_SETTINGS_H
