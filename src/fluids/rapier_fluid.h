#ifndef RAPIER_FUILD_H
#define RAPIER_FUILD_H

#include <godot_cpp/classes/sprite2d.hpp>

namespace godot {

class RapierFluid : public Sprite2D {
	GDCLASS(RapierFluid, Sprite2D)

private:
	double time_passed;

protected:
	static void _bind_methods();

public:
	RapierFluid();
	~RapierFluid();

	void _process(double delta) override;
};

}

#endif // RAPIER_FUILD_H
