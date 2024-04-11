#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

## Libs
if env["dev_build"]:
    lib_folder = "src/rapier2d-wrapper/target/debug"
else:
    lib_folder = "src/rapier2d-wrapper/target/release"
if env["platform"] == "ios":
	env.Append(LINKFLAGS='-framework Security')
	
if env["platform"] == "windows":
	lib_file = "rapier2d_wrapper{}"
	lib = lib_file.format(env["LIBSUFFIX"])
else:
	lib_file = "librapier2d_wrapper{}"
	lib = lib_file.format(env["LIBSUFFIX"])
env.Append(LIBPATH=[lib_folder])
env.Append(LIBS=[lib])

## Sources
env.Append(CPPPATH=["src/"])
sources = [Glob("src/*.cpp"),Glob("src/bodies/*.cpp"),Glob("src/joints/*.cpp"),Glob("src/servers/*.cpp"),Glob("src/shapes/*.cpp"),Glob("src/spaces/*.cpp"),Glob("src/liquids/*.cpp")]

if env["platform"] == "windows":
    env.Append(CPPDEFINES="WINDOWS_ENABLED")

if env["precision"] == "double":
	env.Append(CPPDEFINES=["REAL_T_IS_DOUBLE"])
else:
	env.Append(CPPDEFINES=["REAL_T_IS_FLOAT"])
if env["platform"] == "macos":
	library = env.SharedLibrary(
		"bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d.{}.{}.framework/libphysics_server_rapier2d.{}.{}.dylib".format(
			env["platform"], env["target"], env["platform"], env["target"]
		),
		source=sources,
	)
elif env["platform"] == "ios":
	library = env.SharedLibrary(
		"bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d.{}.{}.framework/libphysics_server_rapier2d.{}.{}.dylib".format(
			env["platform"], env["target"], env["platform"], env["target"]
		),
		source=sources,
	)
else:
	library = env.SharedLibrary(
		"bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
		source=sources,
	)

Default(library)
