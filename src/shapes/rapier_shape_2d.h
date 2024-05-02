#ifndef RAPIER_SHAPE_2D_H
#define RAPIER_SHAPE_2D_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/local_vector.hpp>

#include "../rapier_include.h"

using namespace godot;

class RapierShape2D;

class RapierShapeOwner2D {
public:
	virtual void _shape_changed(RapierShape2D *p_shape) = 0;
	virtual void remove_shape(RapierShape2D *p_shape) = 0;

	virtual ~RapierShapeOwner2D() {}
};

class RapierShape2D {
	RID rid;
	Rect2 aabb;
	bool configured = false;

	HashMap<RapierShapeOwner2D *, int> owners;

	rapier2d::Handle shape_handle = rapier2d::invalid_handle();

protected:
	void configure(const Rect2 &p_aabb);

	virtual rapier2d::Handle create_rapier_shape() const = 0;

public:
	void destroy_rapier_shape();
	_FORCE_INLINE_ void set_rid(const RID &p_rid) { rid = p_rid; }
	_FORCE_INLINE_ RID get_rid() const { return rid; }

	virtual PhysicsServer2D::ShapeType get_type() const = 0;

	virtual bool allows_one_way_collision() const { return true; }

	rapier2d::Handle get_rapier_shape();

	_FORCE_INLINE_ Rect2 get_aabb(Vector2 origin = Vector2()) const {
		Rect2 aabb_clone = aabb;
		aabb_clone.position += origin;
		return aabb_clone;
	}
	_FORCE_INLINE_ bool is_configured() const { return configured; }

	void add_owner(RapierShapeOwner2D *p_owner);
	void remove_owner(RapierShapeOwner2D *p_owner);
	bool is_owner(RapierShapeOwner2D *p_owner) const;
	const HashMap<RapierShapeOwner2D *, int> &get_owners() const;

	virtual void set_data(const Variant &p_data) = 0;
	virtual Variant get_data() const = 0;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const = 0;

	RapierShape2D() {}
	virtual ~RapierShape2D();
};

#endif // RAPIER_SHAPE_2D_H
