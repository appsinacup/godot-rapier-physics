#include "fluid_2d.h"
#include "../servers/rapier_physics_server_2d.h"
#include <godot_cpp/classes/world2d.hpp>

RapierPhysicsServer2D *_get_rapier_physics_server() {
	static auto *physics_server = dynamic_cast<RapierPhysicsServer2D *>(PhysicsServer2D::get_singleton());
	ERR_FAIL_NULL_V(physics_server, nullptr);

	return physics_server;
}

real_t Fluid2D::get_density() const {
	return density;
}

void Fluid2D::set_density(real_t p_density) {
	if (density != p_density) {
		density = p_density;
		_get_rapier_physics_server()->fluid_set_density(rid, density);
	}
}

PackedVector2Array Fluid2D::get_accelerations() const {
	return _get_rapier_physics_server()->fluid_get_accelerations(rid);
}
PackedVector2Array Fluid2D::get_velocities() const {
	return _get_rapier_physics_server()->fluid_get_velocities(rid);
}
PackedVector2Array Fluid2D::get_points() const {
	return _get_rapier_physics_server()->fluid_get_points(rid);
}
void Fluid2D::set_points(PackedVector2Array p_points) {
	points = p_points;
	Transform2D gl_transform = get_global_transform();
	for (int i = 0; i < p_points.size(); i++) {
		p_points[i] = gl_transform.xform(p_points[i]);
	}
	_get_rapier_physics_server()->fluid_set_points(rid, p_points);
}

RID Fluid2D::get_rid() const {
	return rid;
}

void Fluid2D::set_effects(const TypedArray<FluidEffect2D> &p_effects) {
	effects = p_effects;
	_get_rapier_physics_server()->fluid_set_effects(rid, effects);
}

TypedArray<FluidEffect2D> Fluid2D::get_effects() const {
	return effects;
}

void Fluid2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_accelerations"), &Fluid2D::get_accelerations);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "accelerations"), "", "get_accelerations");
	ClassDB::bind_method(D_METHOD("get_velocities"), &Fluid2D::get_velocities);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "velocities"), "", "get_velocities");
	ClassDB::bind_method(D_METHOD("get_points"), &Fluid2D::get_points);
	ClassDB::bind_method(D_METHOD("set_points", "points"), &Fluid2D::set_points);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "points"), "set_points", "get_points");

	ClassDB::bind_method(D_METHOD("get_density"), &Fluid2D::get_density);
	ClassDB::bind_method(D_METHOD("set_density", "density"), &Fluid2D::set_density);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, U"0,1,or_greater"), "set_density", "get_density");

	ClassDB::bind_method(D_METHOD("get_rid"), &Fluid2D::get_rid);

	ClassDB::bind_method(D_METHOD("get_effects"), &Fluid2D::get_effects);
	ClassDB::bind_method(D_METHOD("set_effects", "effects"), &Fluid2D::set_effects);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "effects", PROPERTY_HINT_TYPE_STRING,
						 String::num(Variant::OBJECT) + "/" + String::num(PROPERTY_HINT_RESOURCE_TYPE) + ":FluidEffect2D"),
			"set_effects", "get_effects");
}

Fluid2D::Fluid2D() {
	rid = _get_rapier_physics_server()->fluid_create();
}

Fluid2D::~Fluid2D() {
	_get_rapier_physics_server()->free_rid(rid);
}

void Fluid2D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
		case NOTIFICATION_WORLD_2D_CHANGED:
		case NOTIFICATION_TRANSFORM_CHANGED: {
			RID space = get_world_2d()->get_space();
			_get_rapier_physics_server()->fluid_set_space(rid, space);
			set_points(points);
			//set_effects(effects);
		} break;

		case NOTIFICATION_EXIT_TREE: {
			_get_rapier_physics_server()->fluid_set_space(rid, RID());
		} break;
	}
}
