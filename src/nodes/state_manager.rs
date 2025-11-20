use godot::classes::CollisionObject2D;
use godot::classes::CollisionObject3D;
use godot::classes::Joint2D;
use godot::classes::Joint3D;
use godot::classes::Marshalls;
use godot::prelude::*;
use hashbrown::HashMap;
use hashbrown::HashSet;
use rapier::geometry::ColliderPair;

use crate::bodies::exportable_object::ExportToImport;
use crate::bodies::exportable_object::ExportableObject;
use crate::bodies::exportable_object::ObjectExportState;
use crate::bodies::exportable_object::ObjectImportState;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::servers::RapierPhysicsServer;
use crate::servers::rapier_physics_singleton::physics_data;
use crate::spaces::rapier_space::SpaceExport;
use crate::spaces::rapier_space::SpaceImport;
use crate::types::PhysicsServer;
use crate::types::SerializationFormat;
use crate::types::bin_to_packed_byte_array;
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
struct CollatedObjectExportState<'a> {
    state: ObjectExportState<'a>,

    // Shape data variables use a string key for json serialization purposes,
    // but actually it's just a stringified i32.
    // Note that these shapes _might_ exist purely on the physics server; there's no guarantee that they'll have shape nodes in the scene.
    shape_owners: HashMap<String, Vec<ObjectExportState<'a>>>,
}
impl<'a> CollatedObjectExportState<'a> {
    pub fn into_collated_import(self) -> CollatedObjectImportState {
        let mut import_state_map = HashMap::new();
        for (key, val) in self.shape_owners {
            let mut shape_imports: Vec<ObjectImportState> = Vec::new();
            // Move the export states out.
            for export_state in val {
                shape_imports.push(export_state.into_import());
            }
            import_state_map.insert(key.to_string(), shape_imports);
        }
        CollatedObjectImportState {
            state: self.state.into_import(),
            shape_owners: import_state_map,
        }
    }
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize, Clone))]
struct CollatedObjectImportState {
    state: ObjectImportState,
    shape_owners: HashMap<String, Vec<ObjectImportState>>,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
// The entire state of the physics server, with physics node states stored too.
struct RawExportState<'a> {
    root_node: String,
    rapier_space: SpaceExport<'a>,
    physics_server_id: i64,
    physics_objects_state: HashMap<String, CollatedObjectExportState<'a>>,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize, Clone))]
struct RawImportState {
    root_node: String,
    rapier_space: SpaceImport,
    physics_server_id: i64,
    physics_objects_state: HashMap<String, CollatedObjectImportState>,
}
impl RawImportState {
    // Destroys (takes ownership of) the export object.
    fn from_export_state(export_state: RawExportState) -> Self {
        RawImportState {
            root_node: export_state.root_node.clone(),
            rapier_space: *export_state.rapier_space.into_import(), // Unbox the space state (boxed to keep down enum size)
            physics_server_id: export_state.physics_server_id,
            physics_objects_state: {
                let mut phys_objs = HashMap::new();
                for (key, val) in export_state.physics_objects_state {
                    phys_objs.insert(key.clone(), val.into_collated_import());
                }
                phys_objs
            },
        }
    }
}
struct CachedState {
    state: RawImportState,
    tag: Variant,
}
#[derive(GodotClass)]
#[class(base = Node)]
struct StateManager {
    #[export]
    root_node: Option<Gd<Node>>,
    base: Base<Node>,
    cached_states: Vec<CachedState>,
    #[export]
    #[var(get = get_max_cache_length, set = set_max_cache_length)]
    max_cache_length: u32,
    #[export]
    rolling_cache: bool,
}
#[godot_api]
impl INode for StateManager {
    fn init(base_in: Base<Node>) -> Self {
        StateManager {
            root_node: None,
            base: base_in,
            cached_states: Vec::new(),
            max_cache_length: 10,
            rolling_cache: false,
        }
    }
}
#[godot_api]
impl StateManager {
    /*---------------------------------------
      GODOT-EXPOSED METHODS
    ---------------------------------------*/
    // CACHE HANDLING: If a user wants to store some small number of states in memory for fast reloads, they can use these cache functionalities.
    #[func]
    fn get_max_cache_length(&self) -> i32 {
        self.max_cache_length as i32
    }

    #[func]
    fn set_max_cache_length(&mut self, new_max_length: u32) {
        self.max_cache_length = new_max_length;
        // If the current cache is longer than max, drain the oldest elements until it's not.
        let excess = self
            .cached_states
            .len()
            .saturating_sub(self.max_cache_length as usize);
        if excess > 0 {
            godot_warn!(
                "Count of cached states exceeds the new maximum limit; the oldest {} states will be removed.",
                excess
            );
            self.cached_states.drain(..excess);
        }
    }

    #[func]
    // A method to fetch all the tags in the cache; this lets the user load from cached states at their discretion.
    // The return vector has the same order as the cached states vector.
    fn peek_cache_tags(&self) -> Vec<Variant> {
        self.cached_states.iter().map(|s| s.tag.clone()).collect()
    }

    #[func]
    fn clear_cache(&mut self) {
        self.cached_states.clear();
    }

    #[func]
    // Can submit an index of -1 to remove the last cached state.
    fn remove_cache_by_index(&mut self, at_index: i32) {
        let usize_index = {
            if at_index < -1 {
                godot_error!(
                    "Tried to remove invalid negative index from cache: {}",
                    at_index
                );
                return;
            } else if at_index == -1 {
                self.cached_states.len() - 1
            } else {
                at_index as usize
            }
        };
        if usize_index >= self.cached_states.len() {
            godot_error!(
                "Specified cache index out of bounds; index is {} but cache length is {}!",
                usize_index,
                self.cached_states.len()
            );
            return;
        }
        self.cached_states.remove(usize_index as usize);
    }

    #[func]
    // A cached state has a tag associated with it; this tag can be any variant type.
    // Tags are not unique, but are the only way for a user to associate some custom item with their cached state.
    fn cache_state_with_tag(&mut self, in_space: Rid, tag: Variant) {
        if let Some(export_state) = self.fetch_state(in_space) {
            let cached_state = CachedState {
                state: RawImportState::from_export_state(export_state),
                tag,
            };
            self.push_state_to_cache(cached_state);
        }
    }

    #[func]
    // A variation with no tag.
    fn cache_state(&mut self, in_space: Rid) {
        if let Some(export_state) = self.fetch_state(in_space) {
            let cached_state = CachedState {
                state: RawImportState::from_export_state(export_state),
                tag: Variant::nil(),
            };
            self.push_state_to_cache(cached_state);
        }
    }

    #[func]
    // Loads the nth state from the cache. -1 grabs last entry. Only the subset of the cache with a matching tag will be considered.
    fn load_cached_state_with_tag(
        &mut self,
        in_space: Rid,
        mut index: i32,
        with_tag: Variant,
    ) -> bool {
        // If no tag was provided, consider our subset to be all our cached states.
        let subset: Vec<&CachedState> = {
            if with_tag.is_nil() {
                self.cached_states.iter().collect()
            } else {
                // Otherwise, if a tag was provided, our subset is the cache members that match the tag.
                self.cached_states
                    .iter()
                    .filter(|c| c.tag == with_tag)
                    .collect()
            }
        };
        if subset.is_empty() {
            return false;
        };
        if index == -1 {
            index = (subset.len() as i32) - 1;
        }
        if let Some(cached_state) = subset.get(index as usize) {
            self.load_state_internal(in_space, cached_state.state.clone());
            return true;
        }
        false
    }

    #[func]
    // Variation with no tag.
    fn load_cached_state(&mut self, in_space: Rid, mut index: i32) -> bool {
        if self.cached_states.is_empty() {
            return false;
        };
        if index == -1 {
            index = (self.cached_states.len() as i32) - 1;
        }
        if let Some(cached_state) = self.cached_states.get(index as usize) {
            self.load_state_internal(in_space, cached_state.state.clone());
            return true;
        }
        false
    }

    // STATE EXPORT/IMPORT: Methods for serializing state for storage on disk.
    #[func]
    // Exports state to serialized variant; for "None" encoding, it outputs a stringified JSON dict. For "GodotBase64",
    // it outputs a GodotBase64 string. For RustBincode, it exports binary. This is mostly intended for saving states to file,
    // but exporting state to JSON can also be useful for debugging.
    fn export_state(&mut self, in_space: Rid, format: SerializationFormat) -> Variant {
        self.serialize_state(in_space, &format)
    }

    #[func]
    // Import any of the allowed variant-type serialized states.
    fn import_state(&mut self, in_space: Rid, load_state: Variant) {
        let loaded_state: RawImportState = match load_state.get_type() {
            VariantType::PACKED_BYTE_ARRAY => {
                let pba = match load_state.try_to::<PackedByteArray>() {
                    Ok(p) => p,
                    Err(_) => {
                        godot_error!(
                            "Variant claims to be a PackedByteArray but failed to convert!"
                        );
                        return;
                    }
                };
                match bincode::deserialize(pba.as_slice()) {
                    Ok(s) => s,
                    Err(err) => {
                        godot_error!("Failed to deserialize state from binary: {}", err);
                        return;
                    }
                }
            }
            VariantType::STRING => {
                match load_state.try_to::<String>() {
                    Ok(as_string) => {
                        // Switch based on whether this is regular JSON or GodotBase64.
                        match serde_json::from_str::<RawImportState>(&as_string) {
                            Ok(s) => s,
                            Err(_) => {
                                // If we weren't able to deserialize this string directly, it might be a GodotBase64 string.
                                let decoded_string = Marshalls::singleton()
                                    .base64_to_utf8(&as_string)
                                    .to_string();
                                match serde_json::from_str::<RawImportState>(&decoded_string) {
                                    Ok(s) => s,
                                    Err(err) => {
                                        godot_error!(
                                            "Failed to deserialize state from JSON string; tried plain JSON and GodotBase64. Error: {}",
                                            err
                                        );
                                        return;
                                    }
                                }
                            }
                        }
                    }
                    Err(err) => {
                        godot_error!("Failed to load state string! Error: {}", err);
                        return;
                    }
                }
            }
            _ => {
                godot_error!(
                    "Loaded state is not a valid type! Must be either PackedByteArray for binary or string for JSON/GodotBase64."
                );
                return;
            }
        };
        // Now we have our imported state. Load it up:
        self.load_state_internal(in_space, loaded_state);
    }

    /*---------------------------------------
      INTERNAL METHODS
    ---------------------------------------*/
    fn check_space(
        collision_object_rid: Rid,
        space_rid: Rid,
        collision_objects: &HashMap<Rid, RapierCollisionObject>,
        physics_objects_ids: &HashMap<u64, Rid>,
    ) -> bool {
        if space_rid != Rid::Invalid
            && let Some(collision_object) = collision_objects.get(&collision_object_rid)
        {
            return collision_object.get_base().get_space(physics_objects_ids) == space_rid;
        };
        false
    }

    fn push_state_to_cache(&mut self, cached_state: CachedState) {
        if self.cached_states.len() < self.max_cache_length as usize {
            self.cached_states.push(cached_state);
            return;
        }
        if self.rolling_cache {
            self.cached_states.remove(0);
            self.cached_states.push(cached_state);
        } else {
            godot_error!(
                "Attempted to cache state, but max number of cached states has been reached; either manually delete an existing state, or enable rolling cache to automatically delete oldest cached state."
            );
        }
    }

    fn load_state_internal(&mut self, space_rid: Rid, mut loaded_state: RawImportState) {
        let physics_data = physics_data();
        let Some(space) = physics_data.spaces.get_mut(&space_rid) else {
            godot_error!("Provided RID didn't correspond to a valid space!");
            return;
        };
        //------------------------------------------------
        // The complexity here comes from the fact that area intersections aren't handled in a very convenient way by Godot;
        // we update Godot's knowledge of area intersections by sending out events whenever Rapier reports the beginning or end of an intersection.
        // But these intersection changes only get emitted when Rapier detects a change in the narrowphase intersection graph-- notably, if we're loading
        // a saved state with a different intersection graph, this won't get flagged by Rapier, so no event is dispatched to Godot.
        // To get around this, I've injected each area's state with information about colliders it's currently monitoring.
        // The flow is like this:
        // 1) Via the space state, compare pre-load narrowphase to post-load narrowphase. This will tell us which areas have stale or new intersections.
        // 2) For any areas with stale intersections, dispatch events to Godot to close out those stale intersections BEFORE loading the area's state.
        // 3) Import the space state and step once to update all the physics object's positions.
        // 4) Load the areas' states (along with all the other physics objects).
        // 5) After loading, go through the areas' new states and emit events for any monitored collider pairs that are in the new intersections list.
        //------------------------------------------------
        // 1) So first, we get our intersection deltas.
        let mut stale_intersections: Vec<ColliderPair> = Vec::new();
        let mut new_intersections: Vec<ColliderPair> = Vec::new();
        if let Some((stale, new)) = space.get_intersection_deltas(
            &mut physics_data.physics_engine,
            &loaded_state.rapier_space.world.narrow_phase,
        ) {
            stale_intersections = stale;
            new_intersections = new;
        } else {
            godot_error!("Failed to gather narrowphase deltas!");
            return;
        }
        // 2) Close the stale intersections for our areas.
        // Is there a better way to iterate through the areas of this specific space?
        for (_, collision_object) in physics_data.collision_objects.iter_mut() {
            if collision_object.get_base().get_space_id() == space.get_state().get_id()
                && let Some(area) = collision_object.get_mut_area()
            {
                area.close_stale_contacts(space, &stale_intersections);
            }
        }
        space.flush();
        // 3) Update space and step 0.
        // Note here that because Space state has a much larger minimum size than other variants, we store the state in a box to keep
        // the enum size down; as such we need to box up the raw state before we can pass it to methods expecting an ObjectImportState.
        let space_state = ObjectImportState::Space(Box::new(loaded_state.rapier_space));
        space.import_state(&mut physics_data.physics_engine, space_state);
        // Zero-step to update our contact graphs. Then flush to broadcast any relevant event signals.
        RapierPhysicsServer::space_step(space_rid, 0.0);
        space.flush();
        // 4) Now we start loading all the objects' states.
        // We can do this by first iterating through all the physics objects in the scene tree; whenever we find an object
        // with state in our loaded state, we apply the state and remove it from our state map.
        // That way, any remaining states come from objects we need to re-instantiate.
        // Similarly, if we fail to find an entry in our loaded state for any given physics node, we should delete that node from the tree...
        let Some(root_node) = self.base().try_get_node_as::<Node>(&loaded_state.root_node) else {
            godot_error!("Couldn't find the root node specified in imported state!");
            return;
        };
        // These nodepaths are relative to the root node.
        let physics_nodes = StateManager::get_all_physics_nodes_in_space(&root_node, space_rid);
        // Store all our node keys here. When we load state for a node, we can delete that node from this set.
        // This should be faster than removing the traversed elements from the objects_state map.
        let mut nodes_pending_load: HashSet<String> =
            loaded_state.physics_objects_state.keys().cloned().collect();
        let mut nodes_to_remove: HashSet<String> = HashSet::new();
        // Iterate through the nodes in our current scene tree.
        for nodepath_str in physics_nodes {
            // If this node doesn't exist in our saved states, then we'll have to remove it.
            if !nodes_pending_load.contains(&nodepath_str) {
                nodes_to_remove.insert(nodepath_str);
                continue;
            }
            let full_path_string = loaded_state.root_node.clone().to_string() + "/" + &nodepath_str;
            let nodepath = NodePath::from(&full_path_string);
            // I think if the body's shapes have changed, this won't necessarily catch it smoothly.
            if let Some(object_state_with_shapes) =
                loaded_state.physics_objects_state.remove(&nodepath_str)
            {
                let object_state = object_state_with_shapes.state;
                if let Some(co2d) = self.base().try_get_node_as::<CollisionObject2D>(&nodepath) {
                    let this_rid = co2d.get_rid();
                    // Load the object's personal state (doesn't include shapes):
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);
                    // Now get the shape owner data.
                    let shape_owner_states = object_state_with_shapes.shape_owners;
                    for (shape_owner_number, mut shape_states) in shape_owner_states {
                        let shape_owner_id = match shape_owner_number.parse() {
                            Ok(i) => i,
                            Err(_) => {
                                godot_error!(
                                    "LoadState Failed to parse shape owner ID! Skipping shape state."
                                );
                                continue;
                            }
                        };
                        for i in 0..shape_states.len() {
                            // Yeah, see, this part grabs the shape from the existing node, which isn't reliable (what if the shapes have changed?).
                            // Since the shape import literally deletes and recreates the shape anyway, we should just clear all the shape owners and rebuild.
                            if let Some(shape) =
                                co2d.shape_owner_get_shape(shape_owner_id, i as i32)
                            {
                                RapierPhysicsServer::load_state_internal(
                                    shape.get_rid(),
                                    shape_states.remove(i),
                                );
                            };
                        }
                    }
                }
                // Maybe could be tidier as a macro, to avoid duplication between 2D and 3D.
                else if let Some(co3d) =
                    self.base().try_get_node_as::<CollisionObject3D>(&nodepath)
                {
                    let this_rid = co3d.get_rid();
                    // Load the object's personal state (doesn't include shapes):
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);
                    // Now get the shape owner data.
                    let shape_owner_states = object_state_with_shapes.shape_owners;
                    for (shape_owner_number, mut shape_states) in shape_owner_states {
                        let shape_owner_id = match shape_owner_number.parse() {
                            Ok(i) => i,
                            Err(_) => {
                                godot_error!(
                                    "LoadState Failed to parse shape owner ID! Skipping shape state."
                                );
                                continue;
                            }
                        };
                        for i in 0..shape_states.len() {
                            if let Some(shape) =
                                co3d.shape_owner_get_shape(shape_owner_id, i as i32)
                            {
                                RapierPhysicsServer::load_state_internal(
                                    shape.get_rid(),
                                    shape_states.remove(i),
                                );
                                godot_print!("Loaded shape state.");
                            };
                        }
                    }
                } else if let Some(joint2d) = self.base().try_get_node_as::<Joint2D>(&nodepath) {
                    let this_rid = joint2d.get_rid();
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);
                } else if let Some(joint3d) = self.base().try_get_node_as::<Joint3D>(&nodepath) {
                    let this_rid = joint3d.get_rid();
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);
                } else {
                    godot_error!("Attempted to load state of non-physics object.");
                }
                // Remove the node from the pending list.
                nodes_pending_load.remove(&nodepath_str);
            } else {
                // TODO: We'll remove the node later.
                nodes_to_remove.insert(nodepath_str);
            }
        }
        // 5) Open the new intersections for our areas.
        for (_, collision_object) in physics_data.collision_objects.iter_mut() {
            if collision_object.get_base().get_space_id() == space.get_state().get_id()
                && let Some(area) = collision_object.get_mut_area()
            {
                area.open_new_contacts(space, &new_intersections);
            }
        }
        // Flush space queries one last time, to emit the newly opened events.
        space.flush();
        RapierPhysicsServer::set_global_id(loaded_state.physics_server_id);
    }

    // Outputs the whole state of our physics server, along with the states of all the physics nodes.
    fn fetch_state<'a>(&'a mut self, in_space: Rid) -> Option<RawExportState<'a>> {
        if let Some(root_node) = &self.root_node {
            let root_nodepath = root_node.get_path();
            // Each physics node gets its own variant vector. The first entry in this vector will be that node's serialized data;
            // if the node has nested data (eg if it's a 2D or 3D CollisionObject), then it will also have a second entry,
            // which will be a nested dictionary containing data on all its shapes.
            let physics_nodes = StateManager::get_all_physics_nodes_in_space(root_node, in_space);
            let mut object_states: HashMap<String, CollatedObjectExportState> = HashMap::new();
            for nodepath_str in physics_nodes {
                let full_path_string = root_nodepath.to_string() + "/" + &nodepath_str;
                let nodepath = NodePath::from(&full_path_string);
                let collated_state: CollatedObjectExportState = {
                    let mut self_state: Option<ObjectExportState> = None;
                    let mut collated_shape_owners: HashMap<String, Vec<ObjectExportState>> =
                        HashMap::new();
                    if let Some(mut co2d) =
                        self.base().try_get_node_as::<CollisionObject2D>(&nodepath)
                    {
                        let this_rid = co2d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);
                        let shape_owners: Vec<i32> = co2d.get_shape_owners().to_vec();
                        for owner_shape_id in shape_owners.iter() {
                            let mut this_owner_shapes: Vec<ObjectExportState> = Vec::new();
                            let shape_count =
                                co2d.shape_owner_get_shape_count(*owner_shape_id as u32);
                            for i in 0..shape_count {
                                if let Some(shape) =
                                    co2d.shape_owner_get_shape(*owner_shape_id as u32, i)
                                    && let Some(shape_state) =
                                        self.get_physics_node_state(shape.get_rid())
                                {
                                    this_owner_shapes.push(shape_state);
                                }
                            }
                            collated_shape_owners
                                .insert(owner_shape_id.to_string(), this_owner_shapes);
                        }
                    }
                    // Maybe could be tidier as a macro, to avoid duplication between 2D and 3D.
                    else if let Some(mut co3d) =
                        self.base().try_get_node_as::<CollisionObject3D>(&nodepath)
                    {
                        let this_rid = co3d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);
                        let shape_owners: Vec<i32> = co3d.get_shape_owners().to_vec();
                        for owner_shape_id in shape_owners.iter() {
                            let mut this_owner_shapes: Vec<ObjectExportState> = Vec::new();
                            let shape_count =
                                co3d.shape_owner_get_shape_count(*owner_shape_id as u32);
                            for i in 0..shape_count {
                                if let Some(shape) =
                                    co3d.shape_owner_get_shape(*owner_shape_id as u32, i)
                                    && let Some(shape_state) =
                                        self.get_physics_node_state(shape.get_rid())
                                {
                                    this_owner_shapes.push(shape_state);
                                }
                            }
                            collated_shape_owners
                                .insert(owner_shape_id.to_string(), this_owner_shapes);
                        }
                    } else if let Some(joint2d) = self.base().try_get_node_as::<Joint2D>(&nodepath)
                    {
                        let this_rid = joint2d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);
                    } else if let Some(joint3d) = self.base().try_get_node_as::<Joint3D>(&nodepath)
                    {
                        let this_rid = joint3d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);
                    } else {
                        godot_error!("Attempted to serialize state of non-physics object.");
                    }
                    if let Some(self_state) = self_state {
                        CollatedObjectExportState {
                            state: self_state,
                            shape_owners: collated_shape_owners,
                        }
                    } else {
                        godot_error!("Failed to collect state for object.");
                        continue;
                    }
                };
                object_states.insert(nodepath_str, collated_state);
            }
            // Now we've got references to all the owned states for our physics objects (for whatever export format we've selected).
            // For the Godot encodings, that means the state is stored in Variants; for bincode, it's just references to the raw state objects.
            // From this point, we need to build either a Godot dictionary (for JSON or GodotBase4 encoding) or a nested Rust Hashmap, for Bincode.
            let physics_server_index: i64 = {
                match PhysicsServer::singleton().try_cast::<RapierPhysicsServer>() {
                    Ok(physics_singleton) => physics_singleton.bind().implementation.id as i64,
                    Err(_) => 0,
                }
            };
            let Some(space_state) = self.get_physics_node_state(in_space) else {
                godot_error!("Failed to export space state for RID {}!", in_space);
                return None;
            };
            let ObjectExportState::Space(space_state) = space_state else {
                godot_error!(
                    "Exported state for RID {} was not RapierSpace state!",
                    in_space
                );
                return None;
            };
            // So now we have our whole composed raw state, ready to serialize to json or binary.
            let raw_state = RawExportState {
                root_node: root_nodepath.to_string(),
                rapier_space: space_state,
                physics_server_id: physics_server_index,
                physics_objects_state: object_states,
            };
            return Some(raw_state);
        }
        godot_error!("No root node specified for StateManager!");
        None
    }

    // Grabs physics state via fetch_state and serializes it to a specified format.
    fn serialize_state(&mut self, in_space: Rid, format: &SerializationFormat) -> Variant {
        if let Some(raw_state) = self.fetch_state(in_space) {
            match format {
                SerializationFormat::Json | SerializationFormat::GodotBase64 => {
                    let serialized_state = match serde_json::to_value(raw_state) {
                        Ok(v) => v,
                        Err(err) => {
                            godot_error!("Failed to serialize physics state to JSON: {}", err);
                            return Variant::nil();
                        }
                    };
                    // Here we switch on format; for GodotBase64, we let Godot's marshall do all the work. For "none", we just return Serde's JSON string.
                    if matches!(format, SerializationFormat::GodotBase64) {
                        let serialized_string =
                            Marshalls::singleton().utf8_to_base64(&serialized_state.to_string());
                        serialized_string.to_variant()
                    } else {
                        serialized_state.to_string().to_variant()
                    }
                }
                SerializationFormat::RustBincode => {
                    let serialized_state = match bincode::serialize(&raw_state) {
                        Ok(b) => b,
                        Err(err) => {
                            godot_error!("Failed to serialize physics state to binary: {}", err);
                            return Variant::nil();
                        }
                    };
                    bin_to_packed_byte_array(serialized_state).to_variant()
                }
            }
        } else {
            godot_error!("Failed to serialize state.");
            Variant::nil()
        }
    }

    fn is_physics_node(node: &Gd<Node>) -> bool {
        node.is_class("CollisionObject2D")
            || node.is_class("CollisionObject3D")
            || node.is_class("Joint2D")
            || node.is_class("Joint3D")
        //|| node.is_class("CollisionShape2D") || node.is_class("CollisionPolygon2D") || node.is_class("CollisionShape3D") || node.is_class("CollisionPolygon3D")
    }

    fn get_all_physics_nodes_in_space(root_node: &Gd<Node>, in_space: Rid) -> Vec<String> {
        let physics_data = physics_data();
        let mut physics_nodepaths: Vec<String> = Vec::new();
        if !root_node.is_instance_valid() {
            godot_error!("StateManager: Root node is invalid!");
            return physics_nodepaths;
        }
        if StateManager::is_physics_node(root_node) {
            let rid = root_node
                .clone()
                .try_cast::<CollisionObject2D>()
                .ok()
                .map(|n| n.get_rid())
                .or_else(|| {
                    root_node
                        .clone()
                        .try_cast::<CollisionObject2D>()
                        .ok()
                        .map(|n| n.get_rid())
                })
                .or_else(|| {
                    root_node
                        .clone()
                        .try_cast::<Joint2D>()
                        .ok()
                        .map(|n| n.get_rid())
                })
                .or_else(|| {
                    root_node
                        .clone()
                        .try_cast::<Joint3D>()
                        .ok()
                        .map(|n| n.get_rid())
                });
            if let Some(rid) = rid
                && StateManager::check_space(
                    rid,
                    in_space,
                    &physics_data.collision_objects,
                    &physics_data.ids,
                )
            {
                physics_nodepaths.push(root_node.get_name().to_string());
            }
        }
        let all_descendants = StateManager::collect_all_children_recursive(root_node);
        for child in all_descendants {
            if StateManager::is_physics_node(&child) {
                let rid = child
                    .clone()
                    .try_cast::<CollisionObject2D>()
                    .ok()
                    .map(|n| n.get_rid())
                    .or_else(|| {
                        child
                            .clone()
                            .try_cast::<CollisionObject2D>()
                            .ok()
                            .map(|n| n.get_rid())
                    })
                    .or_else(|| {
                        child
                            .clone()
                            .try_cast::<Joint2D>()
                            .ok()
                            .map(|n| n.get_rid())
                    })
                    .or_else(|| {
                        child
                            .clone()
                            .try_cast::<Joint3D>()
                            .ok()
                            .map(|n| n.get_rid())
                    });
                if let Some(rid) = rid
                    && StateManager::check_space(
                        rid,
                        in_space,
                        &physics_data.collision_objects,
                        &physics_data.ids,
                    )
                {
                    physics_nodepaths.push(root_node.get_path_to(&child).to_string());
                }
            }
        }
        physics_nodepaths
    }

    fn collect_all_children_recursive(node: &Gd<Node>) -> Vec<Gd<Node>> {
        let mut descendants = Vec::new();
        for child in node.get_children().iter_shared() {
            descendants.push(child.clone());
            descendants.extend(StateManager::collect_all_children_recursive(&child));
        }
        descendants
    }

    fn get_physics_node_state(&self, rid: Rid) -> Option<ObjectExportState<'_>> {
        if let Some(state) = RapierPhysicsServer::fetch_state_internal(rid) {
            Some(state)
        } else {
            panic!("No physics state found for Rid {:?}", rid);
        }
    }
}
