use godot::{prelude::*, classes::{CollisionObject2D, CollisionObject3D, Joint2D, Joint3D, Marshalls, Json, CollisionShape2D}};
use hashbrown::{HashMap, HashSet};
use serde::de::Error;
use serde_json::Map;

use crate::{servers::{RapierPhysicsServer, rapier_physics_singleton::physics_data, rapier_physics_server_2d::RapierPhysicsServer2D}, 
types::{PhysicsServer, SerializationFormat, bin_to_packed_byte_array}, bodies::{rapier_collision_object::{RapierCollisionObject, IRapierCollisionObject}, 
rapier_area::RapierAreaState, exportable_object::{ExportableObject, ObjectExportState, ObjectImportState}}, spaces::{rapier_space::{SpaceExport, RapierSpace}, rapier_direct_space_state_impl}, shapes::rapier_shape_base::ShapeExport};
use crate::spaces::rapier_space::SpaceImport;

#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize)
)]
struct CollatedObjectExportState<'a> {
    state: ObjectExportState<'a>,

    // Shape data variables use a string key for json serialization purposes,
    // but actually it's just a stringified i32.    
    // Note that these shapes _might_ exist purely on the physics server; there's no guarantee that they'll have shape nodes in the scene.
    shape_owners: HashMap<String, Vec<ObjectExportState<'a>>>,
}

#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Deserialize)
)]
struct CollatedObjectImportState {
    state: ObjectImportState,
    shape_owners: HashMap<String, Vec<ObjectImportState>>,
}

#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize)
)]
struct RawExportState<'a> {
    root_node: String,
    rapier_space: SpaceExport<'a>,
    physics_server_id: i64,
    physics_objects_state: HashMap<String, CollatedObjectExportState<'a>>
}

#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Deserialize)
)]
struct RawImportState {
    root_node: String,
    rapier_space: SpaceImport,
    physics_server_id: i64,
    physics_objects_state: HashMap<String, CollatedObjectImportState>
}

// Grabs the state for a physics object; the object has to implement ExportableObject.
pub fn get_state_for_export<'a>(physics_object: Rid) -> Option<ObjectExportState<'a>> {
    let physics_data = physics_data();
    if let Some(collision_object) = physics_data.collision_objects.get(&physics_object) 
    {
        match collision_object {
            RapierCollisionObject::RapierArea(area) => {
                return Some(ObjectExportState::RapierArea(
                    area.get_export_state(&mut physics_data.physics_engine)?,
                ));
            }
            RapierCollisionObject::RapierBody(body) => {
                return Some(ObjectExportState::RapierBody(
                    body.get_export_state(&mut physics_data.physics_engine)?,
                ));
            }
        }
    }
    use crate::shapes::rapier_shape::IRapierShape;
    if let Some(shape) = physics_data.shapes.get(&physics_object) {
        return Some(ObjectExportState::RapierShapeBase(
            shape.get_base().get_export_state(&mut physics_data.physics_engine)?,
        ));
    }
    use crate::joints::rapier_joint::IRapierJoint;
    if let Some(joint) = physics_data.joints.get(&physics_object) {
        return Some(ObjectExportState::RapierJointBase(
            joint.get_base().get_export_state(&mut physics_data.physics_engine)?,
        ));
    }
    if let Some(space) = physics_data.spaces.get(&physics_object) {
        return Some(ObjectExportState::RapierSpace(
            space.get_export_state(&mut physics_data.physics_engine)?,
        ));
    }
    None
}

#[derive(GodotClass)]
#[class(base = Node)]
struct StateManager {    
    #[export]
    root_node: Option<Gd<Node>>,
    base: Base<Node>,
}

#[godot_api]
impl INode for StateManager {
    fn init(base_in: Base<Node>) -> Self {
        StateManager {
            root_node: None,
            base: base_in,
        }
    }
}

#[godot_api]
impl StateManager {
    fn check_space(
        collision_object_rid: Rid,
        space_rid: Rid,
        collision_objects: &HashMap<Rid, RapierCollisionObject>,
        physics_objects_ids: &HashMap<u64, Rid>,
    ) -> bool {
        if space_rid != Rid::Invalid
        && let Some(collision_object) = collision_objects.get(&collision_object_rid) {
            return collision_object.get_base().get_space(&physics_objects_ids) == space_rid
        };
    
        false
    }

    #[func]
    fn save_state(
        &mut self,
        in_space: Rid,
        format: SerializationFormat,
    ) -> Variant {        
        return self.serialize_state(in_space, &format)
    }

    #[func]
    fn load_state(
        &mut self,
        in_space: Rid,
        load_state: Variant,
    ) {        
        let physics_data = physics_data();
        let Some(space) = physics_data.spaces.get_mut(&in_space) else {
            godot_error!("Provided RID didn't correspond to a valid space!");
            return
        };
      
        let mut loaded_state: RawImportState = match load_state.get_type() {
            VariantType::PACKED_BYTE_ARRAY => {
                let pba = match load_state.try_to::<PackedByteArray>() {
                    Ok(p) => p,
                    Err(_) => {
                        godot_error!("Variant claims to be a PackedByteArray but failed to convert!");
                        return;
                    }
                };

                match bincode::deserialize(pba.as_slice()) {
                    Ok(s) => s,
                    Err(err) => {
                        godot_error!("Failed to deserialize state from binary: {}", err);
                        return
                    }
                }
            },
            VariantType::STRING => {
                match load_state.try_to::<String>() {
                    Ok(as_string) => {                        
                        // Switch based on whether this is regular JSON or GodotBase64.
                        match serde_json::from_str::<RawImportState>(&as_string) {
                            Ok(s) => s,
                            Err(_) => {
                                // If we weren't able to deserialize this string directly, it might be a GodotBase64 string.
                                let decoded_string = Marshalls::singleton().base64_to_utf8(&as_string).to_string();
                                match serde_json::from_str::<RawImportState>(&decoded_string) {
                                    Ok(s) => s,
                                    Err(err) => {
                                        godot_error!("Failed to deserialize state from JSON string; tried plain JSON and GodotBase64. Error: {}", err);
                                        return
                                    }
                                }
                            }
                        }
                    },                   
                    Err(err) => {
                        godot_error!("Failed to load state string! Error: {}", err);
                        return
                    }
                }
            },
            _ => {
                godot_error!("Loaded state is not a valid type! Must be either PackedByteArray for binary or string for JSON/GodotBase64.");
                return
            }
        };

        // Now we have our imported state. First load up the space:
        space.import_state(&mut physics_data.physics_engine, ObjectImportState::RapierSpace(loaded_state.rapier_space));

        // Zero-step to update our contact graphs. Then flush to broadcast any relevant event signals.
        RapierPhysicsServer::space_step(in_space, 0.0);
        space.flush();

        // Now we start loading all the objects' states.
        // We can do this by first iterating through all the physics objects in the scene tree; whenever we find an object
        // with state in our loaded state, we apply the state and remove it from our state map.
        // That way, any remaining states come from objects we need to re-instantiate.
        // Similarly, if we fail to find an entry in our loaded state for any given physics node, we should delete that node from the tree...
        let Some(root_node) = self.base().try_get_node_as::<Node>(&loaded_state.root_node) else {
            godot_error!("Couldn't find the root node specified in imported state!");
            return
        };

        // These nodepaths are relative to the root node.
        let physics_nodes = StateManager::get_all_physics_nodes_in_space(&root_node, in_space);

        // Store all our node keys here. When we load state for a node, we can delete that node from this set.
        // This should be faster than removing the traversed elements from the objects_state map.
        let mut nodes_pending_load: HashSet<String> = loaded_state.physics_objects_state.keys().cloned().collect();
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
            
            if let Some(object_state_with_shapes) = loaded_state.physics_objects_state.remove(&nodepath_str) {
                let object_state = object_state_with_shapes.state;
                if let Some(mut co2d) = self.base().try_get_node_as::<CollisionObject2D>(&nodepath) {
                    let this_rid = co2d.get_rid();

                    // Load the object's personal state (doesn't include shapes):
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);

                    // Now get the shape owner data.
                    let shape_owner_states = object_state_with_shapes.shape_owners;
                    for (shape_owner_number, mut shape_states) in shape_owner_states {                        
                        let shape_owner_id = match shape_owner_number.parse() {
                            Ok(i) => i,
                            Err(_) => {
                                godot_error!("LoadState Failed to parse shape owner ID! Skipping shape state.");
                                continue;
                            },
                        };

                        for i in 0..shape_states.len() {
                            if let Some(shape) = co2d.shape_owner_get_shape(shape_owner_id, i as i32) {                      
                                RapierPhysicsServer::load_state_internal(shape.get_rid(), shape_states.remove(i));
                            };
                        }                        
                    }                     
                }
                
                // Maybe could be tidier as a macro, to avoid duplication between 2D and 3D.
                else if let Some(co3d) = self.base().try_get_node_as::<CollisionObject3D>(&nodepath) {
                    let this_rid = co3d.get_rid();

                    // Load the object's personal state (doesn't include shapes):
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);

                    // Now get the shape owner data.
                    let shape_owner_states = object_state_with_shapes.shape_owners;
                    for (shape_owner_number, mut shape_states) in shape_owner_states {
                        let shape_owner_id = match shape_owner_number.parse() {
                            Ok(i) => i,
                            Err(_) => {
                                godot_error!("LoadState Failed to parse shape owner ID! Skipping shape state.");
                                continue;
                            },
                        };

                        for i in 0..shape_states.len() {
                            if let Some(shape) = co3d.shape_owner_get_shape(shape_owner_id, i as i32) {                                
                                RapierPhysicsServer::load_state_internal(shape.get_rid(), shape_states.remove(i));
                                godot_print!("Loaded shape state.");
                            };
                        }                        
                    }              
                }

                else if let Some(joint2d) = self.base().try_get_node_as::<Joint2D>(&nodepath) {
                    let this_rid = joint2d.get_rid();
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);
                }

                else if let Some(joint3d) = self.base().try_get_node_as::<Joint3D>(&nodepath) {
                    let this_rid = joint3d.get_rid();
                    RapierPhysicsServer::load_state_internal(this_rid, object_state);
                }

                else {
                    godot_error!("Attempted to load state of non-physics object.");
                }
                
                // Remove the node from the pending list.
                nodes_pending_load.remove(&nodepath_str);
            } else {
                // We'll remove the node later.
                nodes_to_remove.insert(nodepath_str);
            }
        }

        RapierPhysicsServer::set_global_id(loaded_state.physics_server_id);
    }

    fn serialize_state(
        &mut self,
        in_space: Rid,
        format: &SerializationFormat,
    ) -> Variant {
        if let Some(root_node) = &self.root_node {
            let physics_data = physics_data();
            let root_nodepath = root_node.get_path();

            // Each physics node gets its own variant vector. The first entry in this vector will be that node's serialized data;
            // if the node has nested data (eg if it's a 2D or 3D CollisionObject), then it will also have a second entry, 
            // which will be a nested dictionary containing data on all its shapes.
            let physics_nodes = StateManager::get_all_physics_nodes_in_space(&root_node, in_space);
            godot_print!("this many physics nodes: {}", physics_nodes.len());
            let mut object_states: HashMap<String, CollatedObjectExportState> = HashMap::new();

            for nodepath_str in physics_nodes {  
                let full_path_string = root_nodepath.to_string() + "/" + &nodepath_str;
                let nodepath = NodePath::from(&full_path_string);

                let collated_state: CollatedObjectExportState = {
                    //let mut owned_states = CollatedObjectState::new();
                    //owned_states.node_path = nodepath_str;
                    let mut self_state: Option<ObjectExportState> = None;
                    let mut collated_shape_owners: HashMap<String, Vec<ObjectExportState>> = HashMap::new();

                    
                    if let Some(mut co2d) = self.base().try_get_node_as::<CollisionObject2D>(&nodepath) {
                        let this_rid = co2d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);

                        let shape_owners: Vec<i32> = co2d.get_shape_owners().to_vec();
                        for owner_shape_id in shape_owners.iter() {
                            let mut this_owner_shapes: Vec<ObjectExportState> = Vec::new();
                            let shape_count = co2d.shape_owner_get_shape_count(*owner_shape_id as u32);
                            for i in 0..shape_count {
                                if let Some(shape) = co2d.shape_owner_get_shape(*owner_shape_id as u32, i)
                                {
                                    if let Some(shape_state) = self.get_physics_node_state(shape.get_rid()) {
                                        this_owner_shapes.push(shape_state);
                                    }
                                }
                            };

                            collated_shape_owners.insert(owner_shape_id.to_string(), this_owner_shapes);
                            //owned_states.shape_owner_states.push((*owner_shape_id, this_owner_shapes));
                        }                          
                    }
                    
                    // Maybe could be tidier as a macro, to avoid duplication between 2D and 3D.
                    else if let Some(mut co3d) = self.base().try_get_node_as::<CollisionObject3D>(&nodepath) {
                        let this_rid = co3d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);
                                                    
                        let shape_owners: Vec<i32> = co3d.get_shape_owners().to_vec();
                        for owner_shape_id in shape_owners.iter() {
                            let mut this_owner_shapes: Vec<ObjectExportState> = Vec::new();
                            let shape_count = co3d.shape_owner_get_shape_count(*owner_shape_id as u32);
                            for i in 0..shape_count {
                                if let Some(shape) = co3d.shape_owner_get_shape(*owner_shape_id as u32, i)
                                {
                                    if let Some(shape_state) = self.get_physics_node_state(shape.get_rid()) {
                                        this_owner_shapes.push(shape_state);  
                                    }  
                                }                            
                            };

                            collated_shape_owners.insert(owner_shape_id.to_string(), this_owner_shapes);
                        }                             
                    }

                    else if let Some(joint2d) = self.base().try_get_node_as::<Joint2D>(&nodepath) {
                        let this_rid = joint2d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);
                    }

                    else if let Some(joint3d) = self.base().try_get_node_as::<Joint3D>(&nodepath) {
                        let this_rid = joint3d.get_rid();
                        self_state = self.get_physics_node_state(this_rid);
                    }

                    else {
                        godot_error!("Attempted to serialize state of non-physics object.");
                    }

                    if let Some(self_state) = self_state {
                        CollatedObjectExportState { state: self_state, shape_owners: collated_shape_owners }
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
                    Ok(physics_singleton) => {
                        physics_singleton.bind().implementation.id as i64
                    }
                    Err(_) => {
                        0
                    }
                }
            };

            let Some(space_state) =  self.get_physics_node_state(in_space) else {
                godot_error!("Failed to export space state for RID {}!", in_space);
                return Variant::nil()
            };

            let ObjectExportState::RapierSpace(space_state) = space_state else {
                godot_error!("Exported state for RID {} was not RapierSpace state!", in_space);
                return Variant::nil()
            };

            let root_nodepath_string = root_nodepath.to_string();

            // So now we have our whole composed raw state, ready to serialize to json or binary.
            let raw_state = RawExportState{
                root_node: root_nodepath.to_string(),
                rapier_space: space_state,
                physics_server_id: physics_server_index,
                physics_objects_state: object_states,
            };

            match format {
                SerializationFormat::None | SerializationFormat::GodotBase64 => {     
                    let serialized_state = match serde_json::to_value(raw_state) {
                        Ok(v) => v,
                        Err(err) => {
                            godot_error!("Failed to serialize physics state to JSON: {}", err);
                            return Variant::nil()
                        }
                    };

                    // Here we switch on format; for GodotBase64, we let Godot's marshall do all the work. For "none", we just return Serde's JSON string.
                    if matches!(format, SerializationFormat::GodotBase64) {
                        //let serialized_string = Marshalls::singleton().variant_to_base64(&full_physics_server_state.to_string().to_variant());
                        let serialized_string = Marshalls::singleton().utf8_to_base64(&serialized_state.to_string());
                        return serialized_string.to_variant()
                    } else {
                        return serialized_state.to_string().to_variant()
                    }                    
                }                
                SerializationFormat::RustBincode => { 
                    let serialized_state = match bincode::serialize(&raw_state) {
                        Ok(b) => b,
                        Err(err) => {
                            godot_error!("Failed to serialize physics state to binary: {}", err);
                            return Variant::nil()
                        }
                    };
                    
                    return bin_to_packed_byte_array(serialized_state).to_variant();
                },
            }

            // That concludes the format-agnostic state gathering. Now, depending on format, we need to start serializing the state.
            // match format {
            //     SerializationFormat::None | SerializationFormat::GodotBase64 => {                    
            //         let mut full_physics_server_state = serde_json::Value::Object(Map::new());

            //         if let serde_json::Value::Object(map) = &mut full_physics_server_state {
            //             let root_node_as_val = match serde_json::to_value(root_nodepath_string) {
            //                 Ok(v) => v,
            //                 Err(err) => {
            //                     godot_error!("Failed to serialize root node path! {}", err);
            //                     return Variant::nil()
            //                 },
            //             };
            //             map.insert("root_node".to_string(), root_node_as_val);

            //             let physics_server_index_as_val = match serde_json::to_value(physics_server_index) {
            //                 Ok(v) => v,
            //                 Err(err) => {
            //                     godot_error!("Failed to serialize physics server index! {}", err);
            //                     return Variant::nil()
            //                 },
            //             };
            //             map.insert("physics_server_id".to_string(), physics_server_index_as_val);

            //             let space_as_json = match serde_json::to_value(space_state) {
            //                 Ok(v) => v,
            //                 Err(err) => {
            //                     godot_error!("Error serializing space state: {}", err);
            //                     return Variant::nil();
            //                 }
            //             };

            //             if let serde_json::Value::Object(space_map) = space_as_json {
            //                 for (key, val) in space_map {
            //                     map.insert(key,val);
            //                 }
            //             } 

            //             // Now we iterate through all our physics objects' states and push them to our map.
            //             let mut physics_objects_state = serde_json::Value::Object(Map::new());

            //             for object_state in object_states {
            //                 // Serialize the state we'e collated.
            //                 let mut object_collated_state_as_json = serde_json::Value::Object(Map::new());

            //                 // Serialize the object's own state.
            //                 let object_self_state = match serde_json::to_value(object_state.self_state) {
            //                     Ok(v) => v,
            //                     Err(err) => {
            //                         godot_error!("Error serializing object state: {}", err);
            //                         continue;
            //                     }
            //                 };

            //                 if let Some(object_collated_state_as_json) = object_collated_state_as_json.as_object_mut() {
            //                     object_collated_state_as_json.insert("state".to_string(), object_self_state);                            
                            
            //                     // Now we iterate through the shape owners and add in their state.
            //                     let mut shape_owner_states = serde_json::Value::Object(Map::new());
            //                     if let Some(shape_owner_states) = shape_owner_states.as_object_mut() {
            //                         for (owner_id, shape_states) in object_state.shape_owner_states {
            //                             // Serialize the shape states.
            //                             let serialized_shape_states = match serde_json::to_value(shape_states) {
            //                                 Ok(v) => v,
            //                                 Err(err) => {
            //                                     godot_error!("Error serializing shape states: {}", err);
            //                                     continue;
            //                                 }
            //                             };

            //                             let string_owner_id = match serde_json::to_string(&owner_id) {
            //                                 Ok(s) => s,
            //                                 Err(err) => {
            //                                     godot_error!("Error serializing shape owner ID: {}", err);
            //                                     continue;
            //                                 }
            //                             };

            //                             shape_owner_states.insert(string_owner_id, serialized_shape_states);
            //                         }     
            //                     } 

            //                     object_collated_state_as_json.insert("shape_owners".to_string(), shape_owner_states);
            //                 }

            //                 if let Some(objects_map) = physics_objects_state.as_object_mut() {
            //                     objects_map.insert(object_state.node_path, object_collated_state_as_json);
            //                 }                                                 
            //             }
                        
            //             // Push our map of all physics objects states to our full-state map.
            //             map.insert("physics_objects_state".to_string(), physics_objects_state);                
            //         }
                   
            //         // Here we switch on format; for GodotBase64, we let Godot's marshall do all the work. For "none", we just return Serde's JSON string.
            //         if matches!(format, SerializationFormat::GodotBase64) {
            //             //let serialized_string = Marshalls::singleton().variant_to_base64(&full_physics_server_state.to_string().to_variant());
            //             let serialized_string = Marshalls::singleton().utf8_to_base64(&full_physics_server_state.to_string());
            //             return serialized_string.to_variant()
            //         } else {
            //             return full_physics_server_state.to_string().to_variant()
            //         }
            //     }                
            //     SerializationFormat::RustBincode => {                    
            //         let mut bin_state: BinaryState;
            //         if let ObjectExportState::RapierSpace(export_space_state) = space_state {
            //             bin_state = BinaryState {
            //                 root_node: root_nodepath_string,
            //                 space: export_space_state,
            //                 physics_server_id: physics_server_index,
            //                 physics_objects_state: HashMap::new(),
            //             };
            //         } else {
            //             godot_error!("Unable to serialize space state to RustBincode!");
            //             return Variant::nil()
            //         }
                    
            //         let mut physics_objects_map: HashMap<String, BinaryPhysObjState> = HashMap::new();

            //         for object_state in object_states {
            //             let Some(self_state) = object_state.self_state else {
            //                 godot_error!("State serialization for {} unsuccessful.", object_state.node_path);
            //                 continue;
            //             };

            //             let mut composed_shape_owners_states: HashMap<i32, Vec<ObjectExportState>> = HashMap::new();
            //             for (shape_owner_id, shape_states) in object_state.shape_owner_states {
            //                 let mut composed_shape_states: Vec<ObjectExportState> = Vec::new();
            //                 for shape_state in shape_states {
            //                     composed_shape_states.push(shape_state);
            //                 }
            //                 composed_shape_owners_states.insert(shape_owner_id, composed_shape_states);
            //             }

            //             let physics_object_state = BinaryPhysObjState {
            //                 state: self_state,
            //                 shapes: composed_shape_owners_states,
            //             };

            //             physics_objects_map.insert(object_state.node_path, physics_object_state);
            //         }

            //         bin_state.physics_objects_state = physics_objects_map;
                    
            //         match bincode::serialize(&bin_state) {
            //             Ok(binary_data) => {
            //                 return bin_to_packed_byte_array(binary_data).to_variant()
            //             }
            //             Err(e) => {
            //                 godot_error!("Failed to serialize area to binary: {}", e);
            //             }
            //         }

            //         Variant::nil()
            //     },
            // }
        } else {
            godot_error!("No root node specified for StateManager!");
            Variant::nil()
        }
    }

    // fn serialize_string(
    //     string: String,
    //     format: SerializationFormat,
    // ) -> Variant {
    //     match format {
    //         SerializationFormat::None => string.to_variant(),
    //         SerializationFormat::GodotBase64 => Marshalls::singleton().variant_to_base64(&string.to_variant()).to_variant(),
    //         SerializationFormat::RustBincode => {
    //             let mut buf = PackedByteArray::new();
    //             match bincode::serialize(&string) {
    //                 Ok(binary_data) => {
    //                     buf.resize(binary_data.len());
    //                     for i in 0..binary_data.len() {
    //                         buf[i] = binary_data[i];
    //                     }
    //                 },
    //                 Err(e) => {
    //                     godot_error!("Failed to serialize area to binary: {}", e);
    //                 }
    //             }
    //             buf.to_variant()
    //         },
    //     }
    // }

    fn is_physics_node(
        node: &Gd<Node>,
    ) -> bool {
        return node.is_class("CollisionObject2D") || node.is_class("CollisionObject3D") || node.is_class("Joint2D") || node.is_class("Joint3D") 
        //|| node.is_class("CollisionShape2D") || node.is_class("CollisionPolygon2D") || node.is_class("CollisionShape3D") || node.is_class("CollisionPolygon3D")
    }

    fn get_all_physics_nodes_in_space(
        root_node: &Gd<Node>,
        in_space: Rid,
    ) -> Vec<String> {
        let physics_data = physics_data();
        let mut physics_nodepaths: Vec<String> = Vec::new();

        if !root_node.is_instance_valid() {
            godot_error!("StateManager: Root node is invalid!");
            return physics_nodepaths
        }

        if StateManager::is_physics_node(root_node) {
            let rid = root_node.clone()
            .try_cast::<CollisionObject2D>().ok().map(|n| { n.get_rid() })
            .or_else(|| root_node.clone().try_cast::<CollisionObject2D>().ok().map(|n| { n.get_rid() }))            
            .or_else(|| root_node.clone().try_cast::<Joint2D>().ok().map(|n| { n.get_rid() }))            
            .or_else(|| root_node.clone().try_cast::<Joint3D>().ok().map(|n| { n.get_rid() }));

            if let Some(rid) = rid 
            && StateManager::check_space(
                rid,
                in_space,
                &physics_data.collision_objects,
                &physics_data.ids,
            ) {
                physics_nodepaths.push(root_node.get_name().to_string());
            }
        }

        let all_descendants = StateManager::collect_all_children_recursive(root_node);
       
        for child in all_descendants {
            if StateManager::is_physics_node(&child) {
                let rid = child.clone()
                .try_cast::<CollisionObject2D>().ok().map(|n| { n.get_rid() })
                .or_else(|| child.clone().try_cast::<CollisionObject2D>().ok().map(|n| { n.get_rid() }))            
                .or_else(|| child.clone().try_cast::<Joint2D>().ok().map(|n| { n.get_rid() }))            
                .or_else(|| child.clone().try_cast::<Joint3D>().ok().map(|n| { n.get_rid() }));

                if let Some(rid) = rid 
                && StateManager::check_space(
                    rid,
                    in_space,
                    &physics_data.collision_objects,
                    &physics_data.ids,
                ) {                    
                    physics_nodepaths.push(root_node.get_path_to(&child).to_string());
                }
            }
        }

        return physics_nodepaths
    }

    fn collect_all_children_recursive(
        node: &Gd<Node>,
    ) -> Vec<Gd<Node>> {
        let mut descendants = Vec::new();

        for child in node.get_children().iter_shared() {
            descendants.push(child.clone());
            descendants.extend(StateManager::collect_all_children_recursive(&child));
        }

        return descendants
    }

    fn get_physics_node_state(
        & self,
        rid: Rid
    ) -> Option<ObjectExportState> {
        if let Some(state) = RapierPhysicsServer::fetch_state_internal(rid) {
            return Some(state);
        } else {
            panic!("No physics state found for Rid {:?}", rid);
        }
        None
    }

    // fn save_node(
    //     & self,
    //     rid: Rid,
    //     encoding: &SerializationFormat,
    // ) -> ExportStateData {
    //     // Exporting to JSON is a bit awkward; we have our states as json strings from Serde,
    //     // but we need to coerce it into a Godot json object.
        
    //     match encoding {
    //         // In this first case, we want to treat unserialized (eg raw Json) and GodotBase64 string encoding in the same way.
    //         // That essentially means that for GodotBase64, encoding to base64 only happens at the final serialization step.
    //         SerializationFormat::None | SerializationFormat::GodotBase64 => {
    //             if let Some(state) = RapierPhysicsServer::fetch_state_internal(rid) {
    //                 return ExportStateData::SerdeJson(serde_json::json!(state));
    //             } else {
    //                 panic!("No physics state found for Rid {:?}", rid);
    //             }

    //             // if let Some(variant) = serde_json_string_to_variant(RapierPhysicsServer::export_json(rid)) {
    //             //     return StateData::Variant(variant)
    //             // } else {
    //             //     return StateData::Variant(Variant::nil())
    //             // }
    //         },
    //         SerializationFormat::RustBincode => {   
    //             if let Some(state) = RapierPhysicsServer::fetch_state_internal(rid) {
    //                 return ExportStateData::RawState(state);
    //             } else {
    //                 panic!("No physics state found for Rid {:?}", rid);
    //             }
    //         },
    //     }
    // }

    // fn load_node(
    //     &mut self,
    //     rid: Rid,
    //     data: ExportStateData,
    // ) {
    //     match data {
    //         ExportStateData::SerdeJson(json_state) => {
    //             RapierPhysicsServer::import_json(rid, data)
    //         },
    //         ExportStateData::RawState(raw_state) => {

    //         },
    //     }

    //     //RapierPhysicsServer::import_binary(rid, data);
    // }
}