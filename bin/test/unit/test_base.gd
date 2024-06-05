class_name TestBase
extends Node2D

func assert_eq(x, y):
	if absf(x - y) > 0.00001:
		assert(false)
	
