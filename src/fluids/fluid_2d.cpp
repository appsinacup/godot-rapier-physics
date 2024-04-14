#include "fluid_2d.h"
#include "../servers/rapier_physics_server_2d.h"
#include "../servers/rapier_project_settings.h"
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/world2d.hpp>

RapierPhysicsServer2D *_get_rapier_physics_server() {
	auto *physics_server = dynamic_cast<RapierPhysicsServer2D *>(PhysicsServer2D::get_singleton());
	if (!physics_server) {
		ERR_PRINT_ONCE("Fluid2D node requires Rapier2D Physics Engine. Disabling it.");
	}
	return physics_server;
}

real_t Fluid2D::get_density() const {
	return density;
}

void Fluid2D::set_density(real_t p_density) {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return;
	}
	if (density != p_density) {
		density = p_density;
		rapier_physics_server->fluid_set_density(rid, density);
	}
}

PackedVector2Array Fluid2D::get_accelerations() const {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return PackedVector2Array();
	}
	return rapier_physics_server->fluid_get_accelerations(rid);
}
PackedVector2Array Fluid2D::get_velocities() const {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return PackedVector2Array();
	}
	return rapier_physics_server->fluid_get_velocities(rid);
}
PackedVector2Array Fluid2D::get_points() const {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return PackedVector2Array();
	}
	auto new_points = rapier_physics_server->fluid_get_points(rid);

	Transform2D gl_transform = get_global_transform();
	for (int i = 0; i < new_points.size(); i++) {
		new_points[i] = gl_transform.xform_inv(new_points[i]);
	}
	return new_points;
}
PackedVector2Array Fluid2D::create_rectangle_points(int width, int height) {
	PackedVector2Array new_points;
	new_points.resize(width * height);
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			new_points[i + j * width] = Vector2(i * radius * 2, j * radius * 2);
		}
	}
	return new_points;
}
PackedVector2Array Fluid2D::create_circle_points(int p_radius) {
	PackedVector2Array new_points;
	for (float i = -p_radius; i <= p_radius; i += 1) {
		for (float j = -p_radius; j <= p_radius; j += 1) {
			float x = i * radius * 2;
			float y = j * radius * 2;
			if (Math::sqrt(i * i + j * j) <= p_radius) {
				new_points.append(Vector2(x, y));
			}
		}
	}
	return new_points;
}

void Fluid2D::set_points(PackedVector2Array p_points) {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return;
	}
	points = p_points;
	Transform2D gl_transform = get_global_transform();
	for (int i = 0; i < p_points.size(); i++) {
		p_points[i] = gl_transform.xform(p_points[i]);
	}
	rapier_physics_server->fluid_set_points(rid, p_points);
	queue_redraw();
}

RID Fluid2D::get_rid() const {
	return rid;
}

void Fluid2D::set_effects(const TypedArray<FluidEffect2D> &p_effects) {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return;
	}
	effects = p_effects;
	rapier_physics_server->fluid_set_effects(rid, effects);
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

	ClassDB::bind_method(D_METHOD("create_rectangle_points", "width", "height"), &Fluid2D::create_rectangle_points);
	ClassDB::bind_method(D_METHOD("create_circle_points", "radius"), &Fluid2D::create_circle_points);
}

Fluid2D::Fluid2D() {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return;
	}
	rid = rapier_physics_server->fluid_create();
	radius = RapierProjectSettings::get_fluid_particle_radius();
	color = ProjectSettings::get_singleton()->get("debug/shapes/navigation/geometry_face_color");

	debug_draw = RapierProjectSettings::get_fluid_draw_debug();
	if (Engine::get_singleton()->is_editor_hint()) {
		debug_draw = true;
	}
	if (debug_draw) {
		set_process(true);
		set_notify_transform(true);
	}
}

Fluid2D::~Fluid2D() {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return;
	}
	rapier_physics_server->free_rid(rid);
}

void Fluid2D::_notification(int p_what) {
	RapierPhysicsServer2D *rapier_physics_server = _get_rapier_physics_server();
	if (!rapier_physics_server) {
		return;
	}
	switch (p_what) {
		case NOTIFICATION_PROCESS: {
			if (debug_draw) {
				points = get_points();
				queue_redraw();
			}
		} break;
		case NOTIFICATION_ENTER_TREE:
		case NOTIFICATION_WORLD_2D_CHANGED:
		case NOTIFICATION_TRANSFORM_CHANGED:
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED:
		case NOTIFICATION_TRANSLATION_CHANGED: {
			RID space = get_world_2d()->get_space();
			rapier_physics_server->fluid_set_space(rid, space);
			set_points(points);
			if (debug_draw) {
				queue_redraw();
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
			rapier_physics_server->fluid_set_space(rid, RID());
		} break;
		case NOTIFICATION_DRAW: {
			if (debug_draw) {
				points = get_points();
				for (int i = 0; i < points.size(); i++) {
					draw_rect(Rect2(points[i] - Vector2(radius / 2.0, radius / 2.0), Vector2(radius, radius)), color);
				}
			}
		} break;
	}
}
