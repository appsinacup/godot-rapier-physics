#ifndef GDEXTENSION_STATIC_LIBRARY_LOADER_H
#define GDEXTENSION_STATIC_LIBRARY_LOADER_H

#include <functional>

#include "core/extension/gdextension_loader.h"
#include "core/io/config_file.h"
#include "core/os/shared_object.h"

class GDExtensionStaticLibraryLoader : public GDExtensionLoader {
	friend class GDExtensionManager;
	friend class GDExtension;

private:
	String resource_path;
	void *entry_funcptr = nullptr;
	String library_path;
	Vector<SharedObject> library_dependencies;

	HashMap<String, String> class_icon_paths;

public:
	void set_entry_funcptr(void *p_entry_funcptr) { entry_funcptr = p_entry_funcptr; }
	virtual Error open_library(const String &p_path) override;
	virtual Error initialize(GDExtensionInterfaceGetProcAddress p_get_proc_address, const Ref<GDExtension> &p_extension, GDExtensionInitialization *r_initialization) override;
	virtual void close_library() override;
	virtual bool is_library_open() const override;
	virtual bool has_library_changed() const override;
	virtual bool library_exists() const override;
};

#endif // GDEXTENSION_STATIC_LIBRARY_LOADER_H
