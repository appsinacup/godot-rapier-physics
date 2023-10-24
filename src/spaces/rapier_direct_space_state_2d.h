#ifndef RAPIER_DIRECT_SPACE_STATE_2D_H
#define RAPIER_DIRECT_SPACE_STATE_2D_H

#include "../bodies/rapier_area_2d.h"
#include "../bodies/rapier_body_2d.h"
#include "rapier_space_2d.h"

#include <gdextension_interface.h>
#include <godot_cpp/classes/physics_direct_space_state2d.hpp>
#include <godot_cpp/classes/physics_direct_space_state2d_extension.hpp>
#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/physics_server2d_extension_motion_result.hpp>
#include <godot_cpp/classes/physics_server2d_extension_ray_result.hpp>
#include <godot_cpp/classes/physics_server2d_extension_shape_rest_info.hpp>
#include <godot_cpp/classes/physics_server2d_extension_shape_result.hpp>
#include <godot_cpp/classes/physics_test_motion_result2d.hpp>
#include <godot_cpp/templates/hash_set.hpp>

using namespace godot;

class RapierDirectSpaceState2D : public PhysicsDirectSpaceState2DExtension {
	GDCLASS(RapierDirectSpaceState2D, PhysicsDirectSpaceState2DExtension);

protected:
	static void _bind_methods() {}

public:
	RapierSpace2D *space = nullptr;
	virtual int _intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) override;
	virtual bool _intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *r_result) override;
	virtual int _intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) override;
	virtual bool _cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *p_closest_safe, float *p_closest_unsafe) override;
	virtual bool _collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) override;
	virtual bool _rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *r_info) override { return false; }

	RapierDirectSpaceState2D() {}
};

#endif // RAPIER_DIRECT_SPACE_STATE_2D_H
