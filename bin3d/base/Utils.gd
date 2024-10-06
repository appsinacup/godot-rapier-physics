extends Node

func vec2_equals(v1: Vector2, v2: Vector2, tolerance := 0.00001) -> bool:
	return f_equals(v1.x, v2.x, tolerance) and f_equals(v1.y, v2.y, tolerance)

func vec3_equals(v1: Vector3, v2: Vector3, tolerance := 0.00001) -> bool:
	return f_equals(v1.x, v2.x, tolerance) and f_equals(v1.y, v2.y, tolerance) and f_equals(v1.z, v2.z, tolerance)

func f_equals(f1: float, f2: float, tolerance := 0.00001) -> bool:
	return abs(f1 - f2) <= tolerance
