#ifndef RAPIER_COLLISION_OBJECT_2D_H
#define RAPIER_COLLISION_OBJECT_2D_H

#include "../shapes/rapier_shape_2d.h"

#include "../rapier_include.h"
#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/self_list.hpp>

using namespace godot;

class RapierSpace2D;

class RapierCollisionObject2D : public RapierShapeOwner2D {
public:
	enum Type {
		TYPE_AREA,
		TYPE_BODY
	};

private:
	Type type;
	RID rid;
	ObjectID instance_id;
	ObjectID canvas_instance_id;
	bool pickable = true;

	struct Shape {
		Transform2D xform;
		RapierShape2D *shape = nullptr;
		bool disabled = false;
		bool one_way_collision = false;
		real_t one_way_collision_margin = 0.0;
		rapier2d::Handle collider_handle = rapier2d::invalid_handle();
	};

	LocalVector<Shape> shapes;
	RapierSpace2D *space = nullptr;
	Transform2D transform;
	Transform2D inv_transform;
	uint32_t collision_mask = 1;
	uint32_t collision_layer = 1;
	real_t collision_priority = 1.0;
	bool _static = true;

	void _create_shape(Shape &shape, uint32_t p_shape_index);
	void _destroy_shape(Shape &shape, uint32_t p_shape_index);
	void _update_shape_transform(const Shape &shape);

protected:
	rapier2d::Handle body_handle = rapier2d::invalid_handle();
	uint32_t area_detection_counter = 0;

	void _unregister_shapes();

	void _update_transform();

	void _set_static(bool p_static);

	virtual void _init_material(rapier2d::Material &mat) const {}
	virtual void _init_collider(rapier2d::Handle collider_handle) const {}

	virtual void _shapes_changed() = 0;
	void _set_space(RapierSpace2D *p_space);

	RapierCollisionObject2D(Type p_type);

public:
	_FORCE_INLINE_ void set_rid(const RID &p_rid) { rid = p_rid; }
	_FORCE_INLINE_ RID get_rid() const { return rid; }

	_FORCE_INLINE_ void set_instance_id(const ObjectID &p_instance_id) { instance_id = p_instance_id; }
	_FORCE_INLINE_ ObjectID get_instance_id() const { return instance_id; }
	_FORCE_INLINE_ rapier2d::Handle get_body_handle() { return body_handle; }

	_FORCE_INLINE_ void set_canvas_instance_id(const ObjectID &p_canvas_instance_id) { canvas_instance_id = p_canvas_instance_id; }
	_FORCE_INLINE_ ObjectID get_canvas_instance_id() const { return canvas_instance_id; }

	void set_body_user_data(rapier2d::UserData &r_user_data) const;
	static RapierCollisionObject2D *get_body_user_data(const rapier2d::UserData &p_user_data);

	void set_collider_user_data(rapier2d::UserData &r_user_data, uint32_t p_shape_index) const;
	static RapierCollisionObject2D *get_collider_user_data(const rapier2d::UserData &p_user_data, uint32_t &r_shape_index);

	void _shape_changed(RapierShape2D *p_shape) override;

	_FORCE_INLINE_ Type get_type() const { return type; }
	void add_shape(RapierShape2D *p_shape, const Transform2D &p_transform = Transform2D(), bool p_disabled = false);
	void set_shape(int p_index, RapierShape2D *p_shape);
	void set_shape_transform(int p_index, const Transform2D &p_transform);

	_FORCE_INLINE_ int get_shape_count() const { return shapes.size(); }

	_FORCE_INLINE_ RapierShape2D *get_shape(int p_index) const {
		CRASH_BAD_INDEX(p_index, (int)shapes.size());
		return shapes[p_index].shape;
	}

	_FORCE_INLINE_ const Transform2D &get_shape_transform(int p_index) const {
		CRASH_BAD_INDEX(p_index, (int)shapes.size());
		return shapes[p_index].xform;
	}

	void set_transform(const Transform2D &p_transform, bool wake_up = false);

	_FORCE_INLINE_ const Transform2D &get_transform() const { return transform; }
	_FORCE_INLINE_ const Transform2D &get_inv_transform() const { return inv_transform; }

	_FORCE_INLINE_ RapierSpace2D *get_space() const { return space; }

	void set_shape_disabled(int p_index, bool p_disabled);

	_FORCE_INLINE_ bool is_shape_disabled(int p_index) const {
		ERR_FAIL_INDEX_V(p_index, (int)shapes.size(), false);
		return shapes[p_index].disabled;
	}

	_FORCE_INLINE_ void set_shape_as_one_way_collision(int p_idx, bool p_one_way_collision, real_t p_margin) {
		CRASH_BAD_INDEX(p_idx, (int)shapes.size());
		shapes[p_idx].one_way_collision = p_one_way_collision;
		shapes[p_idx].one_way_collision_margin = p_margin;
	}
	_FORCE_INLINE_ bool is_shape_set_as_one_way_collision(int p_idx) const {
		CRASH_BAD_INDEX(p_idx, (int)shapes.size());
		return shapes[p_idx].one_way_collision;
	}

	_FORCE_INLINE_ real_t get_shape_one_way_collision_margin(int p_idx) const {
		CRASH_BAD_INDEX(p_idx, (int)shapes.size());
		return shapes[p_idx].one_way_collision_margin;
	}

	void set_collision_mask(uint32_t p_mask) {
		collision_mask = p_mask;
	}
	_FORCE_INLINE_ uint32_t get_collision_mask() const { return collision_mask; }

	void set_collision_layer(uint32_t p_layer) {
		collision_layer = p_layer;
	}
	_FORCE_INLINE_ uint32_t get_collision_layer() const { return collision_layer; }

	_FORCE_INLINE_ void set_collision_priority(real_t p_priority) {
		ERR_FAIL_COND_MSG(p_priority <= 0, "Priority must be greater than 0.");
		collision_priority = p_priority;
	}
	_FORCE_INLINE_ real_t get_collision_priority() const { return collision_priority; }

	void remove_shape(RapierShape2D *p_shape) override;
	void remove_shape(int p_index);

	virtual void set_space(RapierSpace2D *p_space) = 0;

	_FORCE_INLINE_ bool is_static() const { return _static; }

	void set_pickable(bool p_pickable) { pickable = p_pickable; }
	_FORCE_INLINE_ bool is_pickable() const { return pickable; }

	_FORCE_INLINE_ bool collides_with(RapierCollisionObject2D *p_other) const {
		return p_other->collision_layer & collision_mask;
	}

	_FORCE_INLINE_ bool interacts_with(const RapierCollisionObject2D *p_other) const {
		return collision_layer & p_other->collision_mask || p_other->collision_layer & collision_mask;
	}

	virtual ~RapierCollisionObject2D() {}
};

#endif // RAPIER_COLLISION_OBJECT_2D_H
