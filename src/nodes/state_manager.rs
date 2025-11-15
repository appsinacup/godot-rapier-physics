use godot::{prelude::*, classes::{CollisionObject2D, CollisionObject3D, Joint2D, Joint3D, Marshalls, Json, CollisionShape2D}};
use hashbrown::HashMap;
use serde_json::Map;

use crate::{servers::{RapierPhysicsServer, rapier_physics_singleton::physics_data, rapier_physics_server_2d::RapierPhysicsServer2D}, 
types::{PhysicsServer, SerializationFormat, bin_to_packed_byte_array}, bodies::{rapier_collision_object::{RapierCollisionObject, IRapierCollisionObject}, 
rapier_area::RapierAreaState, exportable_object::{ExportableObject, ObjectExportState}}, spaces::rapier_space::{SpaceExport, RapierSpace}, shapes::rapier_shape_base::ShapeExport};

// enum RapierStateData {
//     A
// }

enum StateData<'a> {
    RawState(ObjectExportState<'a>),
    SerdeJson(serde_json::Value),
    Variant(Variant),
}

impl<'a> StateData<'a> {
    fn into_variant(self) -> Variant {
        match self {
            StateData::Variant(v) => v,
            _ => panic!("called into_variant() on non-Variant StateData"),
        }
    }

    fn take_raw_state(self) -> ObjectExportState<'a> {
        match self {
            StateData::RawState(r) => r,
            _ => panic!("called into_raw_state() on non-rust StateData"),
        }
    }
}

struct CollatedObjectState<'a> {
    node_path: String,
    self_state: StateData<'a>,
    shape_owner_states: Vec<(i32, Vec<StateData<'a>>)>,
}

impl CollatedObjectState<'_> {
    fn new() -> Self {
        Self{
            node_path: String::new(),
            self_state: StateData::Variant(Variant::nil()),
            shape_owner_states: Vec::new(),
        }
    }
}

#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize)
)]
struct BinaryPhysObjState<'a> {
    state: ObjectExportState<'a>,
    shapes: HashMap<i32, Vec<ObjectExportState<'a>>>,
}

#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize)
)]
struct BinaryState<'a> {
    space: SpaceExport<'a>,
    physics_server_id: i64,
    physics_objects_state: HashMap<String, BinaryPhysObjState<'a>>
}

#[derive(Clone)]
pub struct PhysicsObjectRids {
    node_path: String,
    rid: Rid,
    shape_owner_rids: Vec<ShapeOwnerRids>,
}

impl PhysicsObjectRids {
    fn new() -> Self {
        Self {
            node_path: String::new(),
            rid: Rid::Invalid,
            shape_owner_rids: Vec::new(),
        }
    }
}

#[derive(Clone)]
pub struct ShapeOwnerRids {
    owner_id: i32,
    shape_rids: Vec<Rid>,
}

// To avoid mixing serialized and deserialized data in our state (which leads to double serialization during the final state serialization step), 
// we need to convert Serde's json strings into Godot dictionaries/arrays. That way, we can pass out our whole state as a Variant.
fn serde_json_string_to_variant(json_string: String) -> Option<Variant> {
    let mut parsed = Json::new_gd();
    let error = parsed.parse(&json_string);
    
    if error != godot::global::Error::OK {
        godot_error!("Failed to parse JSON string from serde: {:?}", error);
        return None
    }

    return Some(parsed.get_data()) 
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

        match load_state.get_type() {
            VariantType::PACKED_BYTE_ARRAY => {
                godot_print!("Todo");
            },
            VariantType::STRING => {
                match load_state.try_to::<String>() {
                    Ok(as_string) => {

                        match serde_json::from_str::<serde_json::Value>(&as_string) {
                            Ok(serde_json::Value::Object(state_map)) => {
                                godot_print!("Top-level keys: {:?}", state_map.keys());
                                if let Some(space_state) = state_map.get("RapierSpace") {
                                    let space_json_string = serde_json::to_string(space_state)
                                    .expect("Failed to serialize space JSON");
                                    space.import_json(&mut physics_data.physics_engine, space_json_string);
                                }
                            },
                            Ok(other) => {
                                panic!("Expected a JSON object, but got: {:?}", other);
                            }
                            Err(err) => {
                                panic!("Failed to parse JSON: {}", err);
                            }
                        }


                        
                        // Dictionary-ize the string.
                        // let as_json = Json::parse_string(&as_string);
                        // match as_json.try_to::<Dictionary>() {
                        //     Ok(as_dict) => {
                        //         // Grab the space state. We'll need to convert it into a string, and then use serde to deserialize it to space data.
                        //         let as_string = Json::stringify(&as_dict.to_variant()).to_string();
                        //         space.import_json(&mut physics_data.physics_engine, as_string);
                        //     }
                        //     Err(err) => {
                        //         godot_error!("Failed to load state dictionary!");
                        //     }
                        // }
                    }
                    Err(err) => {
                        godot_error!("Failed to load state string!");
                    }
                }
            },
            _ => {
                godot_error!("Loaded state is not a valid variant type!");
            }
        }
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
            let physics_nodes = StateManager::get_all_physics_nodes(&root_node);
            godot_print!("this many physics nodes: {}", physics_nodes.len());
            let mut object_states: Vec<CollatedObjectState> = Vec::new();

            for nodepath_str in physics_nodes {  
                let full_path_string = root_nodepath.to_string() + "/" + &nodepath_str;
                let nodepath = NodePath::from(&full_path_string);

                let collated_state: CollatedObjectState = {
                    let mut owned_states = CollatedObjectState::new();
                    owned_states.node_path = nodepath_str;
                    
                    if let Some(mut co2d) = self.base().try_get_node_as::<CollisionObject2D>(&nodepath) {
                        let this_rid = co2d.get_rid();
                        if !StateManager::check_space(
                            this_rid,
                            in_space,
                            &physics_data.collision_objects,
                            &physics_data.ids,
                        ) { continue; }

                        owned_states.self_state = self.save_node(this_rid, format);
                                                    
                        let shape_owners: Vec<i32> = co2d.get_shape_owners().to_vec();
                        for owner_shape_id in shape_owners.iter() {
                            let mut this_owner_shapes: Vec<StateData> = Vec::new();
                            let shape_count = co2d.shape_owner_get_shape_count(*owner_shape_id as u32);
                            for i in 0..shape_count {
                                if let Some(shape) = co2d.shape_owner_get_shape(*owner_shape_id as u32, i)
                                {
                                    this_owner_shapes.push(self.save_node(shape.get_rid(), format));                                    
                                }                            
                            };

                            owned_states.shape_owner_states.push((*owner_shape_id, this_owner_shapes));
                        }                          
                    }
                    
                    // Maybe could be tidier as a macro, to avoid duplication between 2D and 3D.
                    else if let Some(mut co3d) = self.base().try_get_node_as::<CollisionObject3D>(&nodepath) {
                        let this_rid = co3d.get_rid();
                        if !StateManager::check_space(
                            this_rid,
                            in_space,
                            &physics_data.collision_objects,
                            &physics_data.ids,
                        ) { continue; }

                        owned_states.self_state = self.save_node(this_rid, format);
                                                    
                        let shape_owners: Vec<i32> = co3d.get_shape_owners().to_vec();
                        for owner_shape_id in shape_owners.iter() {
                            let mut this_owner_shapes: Vec<StateData> = Vec::new();
                            let shape_count = co3d.shape_owner_get_shape_count(*owner_shape_id as u32);
                            for i in 0..shape_count {
                                if let Some(shape) = co3d.shape_owner_get_shape(*owner_shape_id as u32, i)
                                {
                                    this_owner_shapes.push(self.save_node(shape.get_rid(), format));
                                }                            
                            };

                            owned_states.shape_owner_states.push((*owner_shape_id, this_owner_shapes));
                        }                             
                    }

                    else if let Some(joint2d) = self.base().try_get_node_as::<Joint2D>(&nodepath) {
                        let this_rid = joint2d.get_rid();
                        if !StateManager::check_space(
                            this_rid,
                            in_space,
                            &physics_data.collision_objects,
                            &physics_data.ids,
                        ) { continue; }

                        owned_states.self_state = self.save_node(this_rid, format);
                    }

                    else if let Some(joint3d) = self.base().try_get_node_as::<Joint3D>(&nodepath) {
                        let this_rid = joint3d.get_rid();
                        if !StateManager::check_space(
                            this_rid,
                            in_space,
                            &physics_data.collision_objects,
                            &physics_data.ids,
                        ) { continue; }

                        owned_states.self_state = self.save_node(this_rid, format);
                    }

                    // else if let Some(mut shape_node) = self.base().try_get_node_as::<CollisionShape2D>(&nodepath) {
                    //     shape_node.
                    // }

                    else {
                        godot_error!("Attempted to serialize state of non-physics object.");
                    }

                    owned_states
                };

                object_states.push(collated_state);
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

            let encoded_space: StateData = self.save_node(in_space, format);// &SerializationFormat::RustBincode);

            match format {
                SerializationFormat::None | SerializationFormat::GodotBase64 => {
                    
                    let mut full_physics_server_state = serde_json::Value::Object(Map::new());

                    if let serde_json::Value::Object(map) = &mut full_physics_server_state {
                        
                        if let StateData::SerdeJson(as_json) = encoded_space {
                            if let serde_json::Value::Object(space_map) = as_json {
                                for (key, val) in space_map {
                                    map.insert(key,val);
                                }
                            }
                            //map.insert("space".to_string(), as_json);
                        }

                        map.insert("physics_server_id".to_string(), physics_server_index.into());

                        
                    }

                    //let mut full_physics_server_state = Dictionary::new();
                    // full_physics_server_state.set("space", encoded_space.into_variant()); //.into_binary()));
                    // full_physics_server_state.set("physics_server_id", physics_server_index.to_variant());

                    // We need to compose the collated object states into a convenient dictionary.
                    let mut physics_objects_state = Dictionary::new();

                    // for object_state in object_states {
                    //     let mut object_dict_entry = Dictionary::new();
                    //     object_dict_entry.set("state", object_state.self_state.into_variant());
                                                    
                    //     let mut shapes_dict = Dictionary::new();
                    //     for shape_owner in object_state.shape_owner_states {
                    //         let mut shape_states: Array<Variant> = Array::new();
                    //         for shape_state in shape_owner.1 {
                    //             shape_states.push(&shape_state.into_variant());
                    //         }
                    //         shapes_dict.set(shape_owner.0, shape_states);
                    //     }
                    //     object_dict_entry.set("shape_owners", shapes_dict);  
                    //     physics_objects_state.set(object_state.node_path, object_dict_entry);                          
                    // }

                    //full_physics_server_state.set("physics_objects_state", physics_objects_state);

                    if matches!(format, SerializationFormat::GodotBase64) {
                        let serialized_string = Marshalls::singleton().variant_to_base64(&full_physics_server_state.to_string().to_variant());
                        return serialized_string.to_variant()
                    } else {
                        return full_physics_server_state.to_string().to_variant()
                    }
                }                
                SerializationFormat::RustBincode => {
                    
                    let mut bin_state: BinaryState;
                    let space_state = encoded_space.take_raw_state();
                    if let ObjectExportState::RapierSpace(export_space_state) = space_state {
                        bin_state = BinaryState {
                            space: export_space_state,
                            physics_server_id: physics_server_index,
                            physics_objects_state: HashMap::new(),
                        };
                    } else {
                        godot_error!("Unable to serialize space state to RustBincode!");
                        return Variant::nil()
                    }
                    
                    let mut physics_objects_map: HashMap<String, BinaryPhysObjState> = HashMap::new();

                    for object_state in object_states {
                        let mut composed_shape_owners_states: HashMap<i32, Vec<ObjectExportState>> = HashMap::new();
                        for (shape_owner_id, shape_states) in object_state.shape_owner_states {
                            let mut composed_shape_states: Vec<ObjectExportState> = Vec::new();
                            for shape_state in shape_states {
                                composed_shape_states.push(shape_state.take_raw_state());
                            }
                            composed_shape_owners_states.insert(shape_owner_id, composed_shape_states);
                        }

                        let physics_object_state = BinaryPhysObjState {
                            state: object_state.self_state.take_raw_state(),
                            shapes: composed_shape_owners_states,
                        };

                        physics_objects_map.insert(object_state.node_path, physics_object_state);
                    }

                    bin_state.physics_objects_state = physics_objects_map;
                    
                    match bincode::serialize(&bin_state) {
                        Ok(binary_data) => {
                            return bin_to_packed_byte_array(binary_data).to_variant()
                        }
                        Err(e) => {
                            godot_error!("Failed to serialize area to binary: {}", e);
                        }
                    }

                    Variant::nil()
                },
            }
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
    }

    fn get_all_physics_nodes(
        root_node: &Gd<Node>,
    ) -> Vec<String> {
        let mut physics_nodepaths: Vec<String> = Vec::new();

        if !root_node.is_instance_valid() {
            godot_error!("StateManager: Root node is invalid!");
            return physics_nodepaths
        }

        if StateManager::is_physics_node(root_node) {
            physics_nodepaths.push(root_node.get_name().to_string());
        }

        let all_descendants = StateManager::collect_all_children_recursive(root_node);
       
        for child in all_descendants {
            if StateManager::is_physics_node(&child) {
                physics_nodepaths.push(root_node.get_path_to(&child).to_string());
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

    fn save_node(
        & self,
        rid: Rid,
        encoding: &SerializationFormat,
    ) -> StateData {
        // Exporting to JSON is a bit awkward; we have our states as json strings from Serde,
        // but we need to coerce it into a Godot json object.
        
        match encoding {
            // In this first case, we want to treat unserialized (eg raw Json) and GodotBase64 string encoding in the same way.
            // That essentially means that for GodotBase64, encoding to base64 only happens at the final serialization step.
            SerializationFormat::None | SerializationFormat::GodotBase64 => {
                if let Some(state) = RapierPhysicsServer::fetch_state_internal(rid) {
                    return StateData::SerdeJson(serde_json::json!(state));
                } else {
                    panic!("No physics state found for Rid {:?}", rid);
                }

                // if let Some(variant) = serde_json_string_to_variant(RapierPhysicsServer::export_json(rid)) {
                //     return StateData::Variant(variant)
                // } else {
                //     return StateData::Variant(Variant::nil())
                // }
            },
            SerializationFormat::RustBincode => {   
                if let Some(state) = RapierPhysicsServer::fetch_state_internal(rid) {
                    return StateData::RawState(state);
                } else {
                    panic!("No physics state found for Rid {:?}", rid);
                }
            },
        }
    }

    fn load_node(
        &mut self,
        rid: Rid,
        data: PackedByteArray
    ) {
        RapierPhysicsServer::import_binary(rid, data);
    }
}