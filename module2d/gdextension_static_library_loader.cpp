#include "gdextension_static_library_loader.h"

#include "core/config/project_settings.h"
#include "core/io/dir_access.h"
#include "core/version.h"
#include "core/extension/gdextension.h"

Error GDExtensionStaticLibraryLoader::open_library(const String &p_path) {
	library_path = p_path;
	return OK;
}

Error GDExtensionStaticLibraryLoader::initialize(GDExtensionInterfaceGetProcAddress p_get_proc_address, const Ref<GDExtension> &p_extension, GDExtensionInitialization *r_initialization) {
	
	GDExtensionInitializationFunction initialization_function = (GDExtensionInitializationFunction)entry_funcptr;
	if (initialization_function == nullptr) {
	}
	GDExtensionBool ret = initialization_function(p_get_proc_address, p_extension.ptr(), r_initialization);

	if (ret) {
		return OK;
	} else {
		ERR_PRINT("GDExtension initialization function '" + library_path + "' returned an error.");
		return FAILED;
	}
}

void GDExtensionStaticLibraryLoader::close_library() {
}

bool GDExtensionStaticLibraryLoader::is_library_open() const {
	return true;
}

bool GDExtensionStaticLibraryLoader::has_library_changed() const {
	return false;
}

bool GDExtensionStaticLibraryLoader::library_exists() const {
	return true;
}
