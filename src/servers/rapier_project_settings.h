#ifndef RAPIER_PROJECT_SETTINGS_H
#define RAPIER_PROJECT_SETTINGS_H

// See /rapier/src/dynamics/integration_parameters.rs for description of parameters
class RapierProjectSettings {
public:
	static void register_settings();

	static bool should_run_on_separate_thread();
	static int get_max_threads();
	static double get_solver_erp();
	static double get_solver_damping_ratio();
	static double get_solver_joint_erp();
	static double get_solver_joint_damping_ratio();
	static double get_solver_allowed_linear_error();
	static double get_solver_max_penetration_correction();
	static double get_solver_prediction_distance();
	static int get_solver_num_solver_iterations();
	static int get_solver_num_additional_friction_iterations();
	static int get_solver_num_internal_pgs_iterations();
	static int get_solver_max_ccd_substeps();
};

#endif // RAPIER_PROJECT_SETTINGS_H
