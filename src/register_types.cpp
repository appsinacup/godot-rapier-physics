#include "register_types.h"

#include <gdextension_interface.h>

#include <godot_cpp/classes/physics_server2d_manager.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/callable.hpp>

#include "bodies/rapier_direct_body_state_2d.h"
#include "servers/rapier_physics_server_2d.h"
#include "servers/rapier_project_settings.h"
#include "spaces/rapier_direct_space_state_2d.h"
#include "spaces/rapier_space_2d.h"

#if defined(WINDOWS_ENABLED)
// Libs needed to link Rapier
#pragma comment(lib, "WS2_32")
#pragma comment(lib, "BCrypt")
#pragma comment(lib, "userenv")
#pragma comment(lib, "advapi32")
#pragma comment(lib, "Ntdll")
#endif

using namespace godot;

static RapierPhysicsServer2DFactory *rapier_2d_factory = nullptr;

void initialize_rapier_2d_module(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			ClassDB::register_class<RapierDirectBodyState2D>();
			ClassDB::register_class<RapierDirectSpaceState2D>();
			ClassDB::register_class<RapierPhysicsServer2D>();
			ClassDB::register_class<RapierPhysicsServer2DFactory>();

			rapier_2d_factory = memnew(RapierPhysicsServer2DFactory());
			PhysicsServer2DManager::get_singleton()->register_server("Rapier2D", Callable(rapier_2d_factory, "create_rapier_2d_callback"));
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
			RapierProjectSettings::register_settings();
		} break;
		default: {
		} break;
	}
}

void uninitialize_rapier_2d_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	memdelete(rapier_2d_factory);
}

extern "C" {

// Initialization.

GDExtensionBool GDE_EXPORT physics_server_rapier_2d_library_init(const GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_rapier_2d_module);
	init_obj.register_terminator(uninitialize_rapier_2d_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}
}
