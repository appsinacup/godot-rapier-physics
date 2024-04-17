#include "../servers/rapier_physics_server_2d.h"
#include "fluid_2d.h"
#include "fluid_effect_2d_elasticity.h"
#include "fluid_effect_2d_surface_tension_akinci.h"
#include "fluid_effect_2d_surface_tension_he.h"
#include "fluid_effect_2d_surface_tension_wcsph.h"
#include "fluid_effect_2d_viscosity_artificial.h"
#include "fluid_effect_2d_viscosity_dfsph.h"
#include "fluid_effect_2d_viscosity_xsph.h"
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
void RapierFluid2D::set_points_and_velocities(PackedVector2Array p_points, PackedVector2Array p_velocities) {
	points = p_points;
	velocities = p_velocities;
	accelerations.resize(points.size());
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));

		rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(points.size() * sizeof(rapier2d::Vector));
		rapier2d::Vector *rapier_velocities = (rapier2d::Vector *)alloca(velocities.size() * sizeof(rapier2d::Vector));
		for (int i = 0; i < points.size(); i++) {
			rapier_points[i] = rapier2d::Vector{ (points[i].x), (points[i].y) };
			rapier_velocities[i] = rapier2d::Vector{ (velocities[i].x), (velocities[i].y) };
		}

		rapier2d::fluid_change_points_and_velocities(space_handle, fluid_handle, rapier_points, points.size(), rapier_velocities);
	}
}

void RapierFluid2D::add_points_and_velocities(PackedVector2Array p_points, PackedVector2Array p_velocities) {
	points.append_array(p_points);
	velocities.append_array(p_velocities);
	accelerations.resize(points.size());
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));

		rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(p_points.size() * sizeof(rapier2d::Vector));
		rapier2d::Vector *rapier_velocities = (rapier2d::Vector *)alloca(p_velocities.size() * sizeof(rapier2d::Vector));
		for (int i = 0; i < p_points.size(); i++) {
			rapier_points[i] = rapier2d::Vector{ (p_points[i].x), (p_points[i].y) };
			rapier_velocities[i] = rapier2d::Vector{ (p_velocities[i].x), (p_velocities[i].y) };
		}

		rapier2d::fluid_add_points_and_velocities(space_handle, fluid_handle, rapier_points, p_points.size(), rapier_velocities);
	}
}



void RapierFluid2D::delete_points(PackedInt32Array p_indices) {
	if (space) {
		size_t *rapier_indexes = (size_t *)alloca(p_indices.size() * sizeof(size_t));

		for (int i = 0; i < p_indices.size(); i++) {
			rapier_indexes[i] = p_indices[i];
			points.remove_at(p_indices[i]);
			velocities.remove_at(p_indices[i]);
			accelerations.remove_at(p_indices[i]);
		}
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));
		rapier2d::fluid_delete_points(space_handle, fluid_handle, rapier_indexes, p_indices.size());
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

void RapierFluid2D::set_effects(const TypedArray<FluidEffect2D> &params) {
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));
		rapier2d::fluid_clear_effects(space_handle, fluid_handle);

		for (int i = 0; i < params.size(); i++) {
			FluidEffect2D *effect = RefCounted::cast_to<FluidEffect2D>(params[i]);
			ERR_FAIL_NULL_MSG(effect, "Parameter must be a FluidEffect2D");
			switch (effect->get_fluid_effect_type()) {
				case FluidEffect2D::FluidEffectType::FLUID_EFFECT_ELASTICITY: {
					FluidEffect2DElasticity *elasticity = static_cast<FluidEffect2DElasticity *>(effect);
					rapier2d::fluid_add_effect_elasticity(space_handle, fluid_handle, elasticity->get_young_modulus(), elasticity->get_poisson_ratio(), elasticity->get_nonlinear_strain());
				} break;
				case FluidEffect2D::FluidEffectType::FLUID_EFFECT_SURFACE_TENSION_AKINCI: {
					FluidEffect2DSurfaceTensionAKINCI *surface_tension = static_cast<FluidEffect2DSurfaceTensionAKINCI *>(effect);
					rapier2d::fluid_add_effect_surface_tension_akinci(space_handle, fluid_handle, surface_tension->get_fluid_tension_coefficient(), surface_tension->get_boundary_adhesion_coefficient());
				} break;
				case FluidEffect2D::FluidEffectType::FLUID_EFFECT_SURFACE_TENSION_HE: {
					FluidEffect2DSurfaceTensionHE *surface_tension = static_cast<FluidEffect2DSurfaceTensionHE *>(effect);
					rapier2d::fluid_add_effect_surface_tension_he(space_handle, fluid_handle, surface_tension->get_fluid_tension_coefficient(), surface_tension->get_boundary_adhesion_coefficient());
				} break;
				case FluidEffect2D::FluidEffectType::FLUID_EFFECT_SURFACE_TENSION_WCSPH: {
					FluidEffect2DSurfaceTensionWCSPH *surface_tension = static_cast<FluidEffect2DSurfaceTensionWCSPH *>(effect);
					rapier2d::fluid_add_effect_surface_tension_wcsph(space_handle, fluid_handle, surface_tension->get_fluid_tension_coefficient(), surface_tension->get_boundary_adhesion_coefficient());
				} break;
				case FluidEffect2D::FluidEffectType::FLUID_EFFECT_VISCOSITY_ARTIFICIAL: {
					FluidEffect2DViscosityArtificial *viscosity = static_cast<FluidEffect2DViscosityArtificial *>(effect);
					rapier2d::fluid_add_effect_viscosity_artificial(space_handle, fluid_handle, viscosity->get_fluid_viscosity_coefficient(), viscosity->get_boundary_viscosity_coefficient());
				} break;
				case FluidEffect2D::FluidEffectType::FLUID_EFFECT_VISCOSITY_DFSPH: {
					FluidEffect2DViscosityDFSPH *viscosity = static_cast<FluidEffect2DViscosityDFSPH *>(effect);
					rapier2d::fluid_add_effect_viscosity_dfsph(space_handle, fluid_handle, viscosity->get_fluid_viscosity_coefficient());
				} break;
				case FluidEffect2D::FluidEffectType::FLUID_EFFECT_VISCOSITY_XSPH: {
					FluidEffect2DViscosityXSPH *viscosity = static_cast<FluidEffect2DViscosityXSPH *>(effect);
					rapier2d::fluid_add_effect_viscosity_xsph(space_handle, fluid_handle, viscosity->get_fluid_viscosity_coefficient(), viscosity->get_boundary_viscosity_coefficient());
				} break;
				default:
					ERR_FAIL_MSG("Unsupported fluid effect type");
			}
		}
	} else {
		if (effects != params) {
			effects = params;
		}
	}
}

void RapierFluid2D::set_space(RapierSpace2D *p_space) {
	if (space == p_space) {
		return;
	}
	// remove from old space
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		if (rapier2d::is_handle_valid_double(fluid_handle)) {
			rapier2d::fluid_destroy(space_handle, fluid_handle);
			fluid_handle = rapier2d::invalid_handle_double();
		}
	}
	space = p_space;
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		if (!rapier2d::is_handle_valid_double(fluid_handle)) {
			fluid_handle = rapier2d::fluid_create(space_handle, density);
			ERR_FAIL_COND(!rapier2d::is_handle_valid_double(fluid_handle));
		}
		set_points(points);
		set_effects(effects);
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

RapierFluid2D::~RapierFluid2D() {
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		if (rapier2d::is_handle_valid_double(fluid_handle)) {
			rapier2d::fluid_destroy(space_handle, fluid_handle);
		}
	}
}
