#ifndef RAPIER_BODY_DIRECT_STATE_2D_H
#define RAPIER_BODY_DIRECT_STATE_2D_H

#include <godot_cpp/classes/physics_direct_body_state2d.hpp>
#include <godot_cpp/classes/physics_direct_body_state2d_extension.hpp>
#include <godot_cpp/classes/physics_server2d.hpp>

using namespace godot;

class RapierBody2D;

class RapierDirectBodyState2D : public PhysicsDirectBodyState2DExtension {
	GDCLASS(RapierDirectBodyState2D, PhysicsDirectBodyState2DExtension);

protected:
	static void _bind_methods() {}

public:
	RapierBody2D *body = nullptr;

	virtual Vector2 _get_total_gravity() const override;
	virtual double _get_total_angular_damp() const override;
	virtual double _get_total_linear_damp() const override;

	virtual Vector2 _get_center_of_mass() const override;
	virtual Vector2 _get_center_of_mass_local() const override;
	virtual double _get_inverse_mass() const override;
	virtual double _get_inverse_inertia() const override;

	virtual void _set_linear_velocity(const Vector2 &p_velocity) override;
	virtual Vector2 _get_linear_velocity() const override;

	virtual void _set_angular_velocity(double p_velocity) override;
	virtual double _get_angular_velocity() const override;

	virtual void _set_transform(const Transform2D &p_transform) override;
	virtual Transform2D _get_transform() const override;

	virtual Vector2 _get_velocity_at_local_position(const Vector2 &p_position) const override;

	virtual void _apply_central_impulse(const Vector2 &p_impulse) override;
	virtual void _apply_impulse(const Vector2 &p_impulse, const Vector2 &p_position = Vector2()) override;
	virtual void _apply_torque_impulse(double p_torque) override;

	virtual void _apply_central_force(const Vector2 &p_force) override;
	virtual void _apply_force(const Vector2 &p_force, const Vector2 &p_position = Vector2()) override;
	virtual void _apply_torque(double p_torque) override;

	virtual void _add_constant_central_force(const Vector2 &p_force) override;
	virtual void _add_constant_force(const Vector2 &p_force, const Vector2 &p_position = Vector2()) override;
	virtual void _add_constant_torque(double p_torque) override;

	virtual void _set_constant_force(const Vector2 &p_force) override;
	virtual Vector2 _get_constant_force() const override;

	virtual void _set_constant_torque(double p_torque) override;
	virtual double _get_constant_torque() const override;

	virtual void _set_sleep_state(bool p_enable) override;
	virtual bool _is_sleeping() const override;

	virtual int _get_contact_count() const override;

	virtual Vector2 _get_contact_local_position(int p_contact_idx) const override;
	virtual Vector2 _get_contact_local_normal(int p_contact_idx) const override;
	virtual int _get_contact_local_shape(int p_contact_idx) const override;

	virtual RID _get_contact_collider(int p_contact_idx) const override;
	virtual Vector2 _get_contact_collider_position(int p_contact_idx) const override;
	virtual uint64_t _get_contact_collider_id(int p_contact_idx) const override;
	virtual int _get_contact_collider_shape(int p_contact_idx) const override;
	virtual Vector2 _get_contact_impulse(int p_contact_idx) const override;

	virtual Vector2 _get_contact_collider_velocity_at_position(int p_contact_idx) const override;

	virtual PhysicsDirectSpaceState2D *_get_space_state() override;

	virtual double _get_step() const override;
};

#endif // RAPIER_BODY_DIRECT_STATE_2D_H
