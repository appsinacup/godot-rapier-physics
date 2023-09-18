#ifndef RAPIER_CONSTRAINT_2D_H
#define RAPIER_CONSTRAINT_2D_H

#include "rapier_body_2d.h"

using namespace godot;

class RapierConstraint2D {
	RapierBody2D **_body_ptr;
	int _body_count;
	bool disabled_collisions_between_bodies = true;

	RID rid;

protected:
	RapierConstraint2D(RapierBody2D **p_body_ptr = nullptr, int p_body_count = 0) {
		_body_ptr = p_body_ptr;
		_body_count = p_body_count;
	}

public:
	_FORCE_INLINE_ void set_rid(const RID &p_rid) { rid = p_rid; }
	_FORCE_INLINE_ RID get_rid() const { return rid; }

	_FORCE_INLINE_ RapierBody2D **get_body_ptr() const { return _body_ptr; }
	_FORCE_INLINE_ int get_body_count() const { return _body_count; }

	_FORCE_INLINE_ void disable_collisions_between_bodies(const bool p_disabled) { disabled_collisions_between_bodies = p_disabled; }
	_FORCE_INLINE_ bool is_disabled_collisions_between_bodies() const { return disabled_collisions_between_bodies; }

	virtual ~RapierConstraint2D() {}
};

#endif // RAPIER_CONSTRAINT_2D_H
