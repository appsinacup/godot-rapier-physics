#ifndef RAPIER_BODY_UTILS_2D_H
#define RAPIER_BODY_UTILS_2D_H

#include <godot_cpp/variant/transform2d.hpp>

using namespace godot;

class RapierBody2D;
class RapierSpace2D;
class RapierShape2D;

class RapierBodyUtils2D
{
public:
    static bool body_motion_recover(
        const RapierSpace2D& p_space,
        const RapierBody2D& p_body,
        Transform2D& p_transform,
        real_t p_margin,
        Vector2& p_recover_motion,
        Rect2& p_body_aabb
    );

    static void cast_motion(
        const RapierSpace2D& p_space,
        const RapierBody2D& p_body,
        const Transform2D& p_transform,
        const Vector2& p_motion,
        const Rect2& p_body_aabb,
        real_t& p_closest_safe,
        real_t& p_closest_unsafe,
        int& p_best_body_shape
    );

    static bool body_motion_collide(
        const RapierSpace2D& p_space,
        const RapierBody2D& p_body,
        const Transform2D& p_transform,
        const Rect2& p_body_aabb,
        int p_best_body_shape,
        real_t p_margin,
        PhysicsServer2DExtensionMotionResult* p_result
    );
};

#endif // RAPIER_BODY_UTILS_2D_H