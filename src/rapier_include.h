#ifndef RAPIER_INCLUDE_H
#define RAPIER_INCLUDE_H

#include "rapier2d-wrapper/includes/rapier2d_wrapper.h"

#include <godot_cpp/templates/hashfuncs.hpp>
#include <godot_cpp/variant/transform2d.hpp>

using namespace godot;

namespace rapier2d {

inline uint32_t handle_hash(Handle handle) {
	uint64_t combined = uint64_t(handle.id) + (uint64_t(handle.generation) << 32);
	return hash_one_uint64(combined);
}

inline uint64_t handle_pair_hash(Handle handle1, Handle handle2) {
	uint64_t hash1 = handle_hash(handle1);
	uint64_t hash2 = handle_hash(handle2);
	return hash1 + (hash2 << 32);
}

inline ShapeInfo shape_info_from_body_shape(rapier2d::Handle shape_handle, const Transform2D &transform) {
	Vector2 origin = transform.get_origin();
	Vector2 scale = transform.get_scale();
	return ShapeInfo{
		shape_handle,
		{ origin.x, origin.y },
		transform.get_rotation(),
		{ scale.x, scale.y },
	};
}

} //namespace rapier2d

#endif // RAPIER_INCLUDE_H
