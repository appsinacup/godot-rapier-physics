#include "../servers/rapier_physics_server_2d.h"
#include "fluid_2d.h"
#include <godot_cpp/classes/world2d.hpp>

real_t RapierFluid2D::get_density() const {
	return density;
}

void RapierFluid2D::set_density(real_t p_density) {
	density = p_density;
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));

		rapier2d::fluid_change_density(space_handle, fluid_handle, density);
	}
}
void RapierFluid2D::set_points(PackedVector2Array p_points) {
	points = p_points;
	velocities.resize(points.size());
	accelerations.resize(points.size());
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));

		rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(points.size() * sizeof(rapier2d::Vector));
		for (int i = 0; i < points.size(); i++) {
			rapier_points[i] = rapier2d::Vector{ (points[i].x), (points[i].y) };
		}

		rapier2d::fluid_change_points(space_handle, fluid_handle, rapier_points, points.size());
	}
}

PackedVector2Array RapierFluid2D::get_points() {
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND_V(!rapier2d::is_handle_valid(space_handle), points);
		ERR_FAIL_COND_V(!rapier2d::is_handle_valid_double(fluid_handle), points);

		rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(points.size() * sizeof(rapier2d::Vector));
		rapier2d::fluid_get_points(space_handle, fluid_handle, rapier_points, points.size());
		for (int i = 0; i < points.size(); i++) {
			points[i] = Vector2(rapier_points[i].x, rapier_points[i].y);
		}
	}
	return points;
}

PackedVector2Array RapierFluid2D::get_velocities() {
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND_V(!rapier2d::is_handle_valid(space_handle), velocities);
		ERR_FAIL_COND_V(!rapier2d::is_handle_valid_double(fluid_handle), velocities);

		rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(points.size() * sizeof(rapier2d::Vector));
		rapier2d::fluid_get_velocities(space_handle, fluid_handle, rapier_points, points.size());
		for (int i = 0; i < points.size(); i++) {
			velocities[i] = Vector2(rapier_points[i].x, rapier_points[i].y);
		}
	}
	return velocities;
}

PackedVector2Array RapierFluid2D::get_accelerations() {
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND_V(!rapier2d::is_handle_valid(space_handle), accelerations);
		ERR_FAIL_COND_V(!rapier2d::is_handle_valid_double(fluid_handle), accelerations);

		rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(points.size() * sizeof(rapier2d::Vector));
		rapier2d::fluid_get_accelerations(space_handle, fluid_handle, rapier_points, points.size());
		for (int i = 0; i < points.size(); i++) {
			accelerations[i] = Vector2(rapier_points[i].x, rapier_points[i].y);
		}
	}
	return accelerations;
}

void RapierFluid2D::set_space(RapierSpace2D *p_space) {
	space = p_space;
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		if (!rapier2d::is_handle_valid_double(fluid_handle)) {
			fluid_handle = rapier2d::fluid_create(space_handle, density);
			ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));
		}
		set_points(points);
	}
}

RapierSpace2D *RapierFluid2D::get_space() const {
	return space;
}

RID RapierFluid2D::get_rid() const {
	return rid;
}
void RapierFluid2D::set_rid(RID p_rid) {
	rid = p_rid;
}
