#include "rapier_shape_2d.h"

void RapierShape2D::configure(const Rect2 &p_aabb) {
	aabb = p_aabb;
	configured = true;
	for (const KeyValue<RapierShapeOwner2D *, int> &E : owners) {
		RapierShapeOwner2D *co = const_cast<RapierShapeOwner2D *>(E.key);
		co->_shape_changed(this);
	}
}

void RapierShape2D::destroy_rapier_shape() {
	if (rapier2d::is_handle_valid(shape_handle)) {
		rapier2d::shape_destroy(shape_handle);
		shape_handle = rapier2d::invalid_handle();
	}
}

rapier2d::Handle RapierShape2D::get_rapier_shape() {
	if (!rapier2d::is_handle_valid(shape_handle)) {
		shape_handle = create_rapier_shape();
	}

	return shape_handle;
}

void RapierShape2D::add_owner(RapierShapeOwner2D *p_owner) {
	HashMap<RapierShapeOwner2D *, int>::Iterator E = owners.find(p_owner);
	if (E) {
		E->value++;
	} else {
		owners[p_owner] = 1;
	}
}

void RapierShape2D::remove_owner(RapierShapeOwner2D *p_owner) {
	HashMap<RapierShapeOwner2D *, int>::Iterator E = owners.find(p_owner);
	ERR_FAIL_COND(!E);
	E->value--;
	if (E->value == 0) {
		owners.remove(E);
	}
}

bool RapierShape2D::is_owner(RapierShapeOwner2D *p_owner) const {
	return owners.has(p_owner);
}

const HashMap<RapierShapeOwner2D *, int> &RapierShape2D::get_owners() const {
	return owners;
}

RapierShape2D::~RapierShape2D() {
	ERR_FAIL_COND(owners.size());
	destroy_rapier_shape();
}
