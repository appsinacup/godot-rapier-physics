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

void Fluid2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_density"), &Fluid2D::get_density);
	ClassDB::bind_method(D_METHOD("set_density", "density"), &Fluid2D::set_density);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, U"0,1,or_greater"), "set_density", "get_density");

	ClassDB::bind_method(D_METHOD("get_rid"), &Fluid2D::get_rid);
}

Fluid2D::Fluid2D() {
	rid = _get_rapier_physics_server()->fluid_create();
}

Fluid2D::~Fluid2D() {
	_get_rapier_physics_server()->free_rid(rid);
}

void Fluid2D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			Transform2D gl_transform = get_global_transform();
			//PhysicsServer2D::get_singleton()->body_set_state(rid, PhysicsServer2D::BODY_STATE_TRANSFORM, gl_transform);

			//bool disabled = !is_enabled();

			//if (!disabled || (disable_mode != DISABLE_MODE_REMOVE)) {
			Ref<World2D> world_ref = get_world_2d();
			ERR_FAIL_COND(!world_ref.is_valid());
			RID space = world_ref->get_space();
			_get_rapier_physics_server()->fluid_set_space(rid, space);
		} break;

		case NOTIFICATION_WORLD_2D_CHANGED: {
			RID space = get_world_2d()->get_space();
			_get_rapier_physics_server()->fluid_set_space(rid, space);
		} break;
	}
}

RID Fluid2D::get_rid() const {
	return rid;
}

PackedVector2Array Fluid2D::get_points() const {
	return points;
}
void Fluid2D::set_points(PackedVector2Array p_points) {
	points = p_points;
}
