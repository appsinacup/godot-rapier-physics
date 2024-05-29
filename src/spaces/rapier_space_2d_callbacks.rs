use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::servers::rapier_physics_singleton_2d::{active_spaces_singleton, bodies_singleton, spaces_singleton};
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::{
    bodies::rapier_collision_object_2d::{
            RapierCollisionObject2D,
        },
    rapier2d::{
        handle::Handle,
        physics_hooks::{CollisionFilterInfo, OneWayDirection},
        physics_world::{
            ActiveBodyInfo, CollisionEventInfo,
            ContactForceEventInfo, ContactPointInfo,
        },
    },
};
use godot::{
    prelude::*,
};
use super::rapier_space_2d::{RapierSpace2D};

pub struct CollidersInfo {
    pub shape1: usize,
    pub object1: Rid,
    pub shape2: usize,
    pub object2: Rid,
}

impl Default for CollidersInfo {
    fn default() -> Self {
        Self { shape1: 0, object1: Rid::Invalid, shape2: 0, object2: Rid::Invalid }
    }
}


impl RapierSpace2D {

    pub fn active_body_callback(active_body_info: &ActiveBodyInfo) {
        let (rid, _) = RapierCollisionObject2D::get_collider_user_data(
            &active_body_info.body_user_data,
        );
        let mut lock = bodies_singleton().lock().unwrap();
        if let Some(body) = lock.collision_objects.get_mut(&rid) {
            if let Some(body) = body.get_mut_body() {
                body.on_marked_active();
            }
        }
    }

    pub fn collision_filter_common_callback(
        filter_info: &CollisionFilterInfo,
        r_colliders_info: &mut CollidersInfo,
    ) -> bool {
        let lock = bodies_singleton().lock().unwrap();
        (r_colliders_info.object1, r_colliders_info.shape1) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data1);
        (r_colliders_info.object2, r_colliders_info.shape2) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data2);
        if let Some(body1) = lock.collision_objects.get(&r_colliders_info.object1) {
            if let Some(body2) = lock.collision_objects.get(&r_colliders_info.object2) {
                return body1.get_base().interacts_with(body2.get_base())
            }
        }
        false
    }

    pub fn collision_filter_body_callback(
        filter_info: &CollisionFilterInfo,
    ) -> bool {
        let mut colliders_info = CollidersInfo::default();
        if !Self::collision_filter_common_callback(filter_info, &mut colliders_info) {
            return false;
        }
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body1) = lock.collision_objects.get(&colliders_info.object1) {
            if let Some(body1) = body1.get_body() {
                if let Some(body2) = lock.collision_objects.get(&colliders_info.object2) {
                    if let Some(body2) = body2.get_body() {
                        if body1.has_exception(body2.get_base().get_rid()) || body2.has_exception(body1.get_base().get_rid()) {
                            return false;
                        }
                    }
            }
        }
        }
    
        true
    }

    pub fn collision_filter_sensor_callback(
        filter_info: &CollisionFilterInfo,
    ) -> bool {
        let mut colliders_info = CollidersInfo::default();
        Self::collision_filter_common_callback(filter_info, &mut colliders_info)
    }

    pub fn collision_modify_contacts_callback(
        filter_info: &CollisionFilterInfo,
    ) -> OneWayDirection {
        let mut result = OneWayDirection::default();
        
        let mut lock = bodies_singleton().lock().unwrap();
        let (object1, shape1) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data1);
        let (object2, shape2) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data2);
        if !lock.collision_objects.contains_key(&object1) || !lock.collision_objects.contains_key(&object2) {
            return result
        }
        let [collision_object_1, collision_object_2] = lock.collision_objects.get_many_mut([&object1, &object2]).unwrap();
        if collision_object_1.get_base().interacts_with(collision_object_2.get_base()) && !collision_object_1.get_base().is_shape_disabled(shape1) && !collision_object_2.get_base().is_shape_disabled(shape2) {
            result.body1 = collision_object_1.get_base().is_shape_set_as_one_way_collision(shape1);
            result.pixel_body1_margin =
                collision_object_1.get_base().get_shape_one_way_collision_margin(shape1);
            result.body2 = collision_object_2.get_base().is_shape_set_as_one_way_collision(shape2);
            result.pixel_body2_margin =
                collision_object_2.get_base().get_shape_one_way_collision_margin(shape2);
            if let Some(body1) = collision_object_1.get_mut_body() {
                if let Some(body2) = collision_object_2.get_mut_body() {
                    let static_linear_velocity1 = body1.get_static_linear_velocity();
                    let static_linear_velocity2 = body2.get_static_linear_velocity();
                    if static_linear_velocity1 != Vector2::default() {
                        body2.to_add_static_constant_linear_velocity(static_linear_velocity1);
                    }
                    if static_linear_velocity2 != Vector2::default() {
                        body1.to_add_static_constant_linear_velocity(static_linear_velocity2);
                    }
                    let static_angular_velocity1 = body1.get_static_angular_velocity();
                    let static_angular_velocity2 = body2.get_static_angular_velocity();
                    if static_angular_velocity1 != 0.0 {
                        body2.to_add_static_constant_angular_velocity(static_angular_velocity1);
                    }
                    if static_angular_velocity2 != 0.0 {
                        body1.to_add_static_constant_angular_velocity(static_angular_velocity2);
                    }
                }
            }
        }

        result
    }

    pub fn collision_event_callback(world_handle: Handle, event_info: &CollisionEventInfo) {
        /*
        let mut spaces_lock = spaces_singleton().lock().unwrap();
        let active_spaces_lock = active_spaces_singleton().lock().unwrap();
        if let Some(space) = active_spaces_lock.active_spaces.get(&world_handle) {
            if let Some(space) = spaces_lock.spaces.get_mut(space) {
                let (mut pObject1, mut shape1) = RapierCollisionObject2D::get_collider_user_data(
                    &event_info.user_data1,
                );
                let (mut pObject2, mut shape2) = RapierCollisionObject2D::get_collider_user_data(
                    &event_info.user_data2,
                );
    
                let mut collider_handle1 = event_info.collider1;
                let mut collider_handle2 = event_info.collider2;

                let (mut rid1, mut rid2) = (Rid::Invalid, Rid::Invalid);
                let (mut instance_id1, mut instance_id2) = (0, 0);
                let (mut type1, mut type2) = (CollisionObjectType::Area, CollisionObjectType::Area);
                

                if event_info.is_removed {
                    if pObject1.is_invalid() {
                        if let Some(removed_collider_info_1) = space.get_removed_collider_info(
                            &collider_handle1,
                        ) {
                            rid1 = removed_collider_info_1.rid;
                            instance_id1 = removed_collider_info_1.instance_id;
                            type1 = removed_collider_info_1.collision_object_type;
                            shape1 = removed_collider_info_1.shape_index;
                        }
                    } else {
                        let body_lock = bodies_singleton().lock().unwrap();
                        if let Some(body) = body_lock.collision_objects.get(&pObject1) {
                            rid1 = body.get_base().get_rid();
                            instance_id1 = body.get_base().get_instance_id();
                            type1 = body.get_base().get_type();
                        }
                    }
                    if pObject2.is_invalid() {
                        if let Some(removed_collider_info_2) = space.get_removed_collider_info(
                            &collider_handle2,
                        ) {
                            rid2 = removed_collider_info_2.rid;
                            instance_id2 = removed_collider_info_2.instance_id;
                            type2 = removed_collider_info_2.collision_object_type;
                            shape2 = removed_collider_info_2.shape_index;
                        }
                    } else {
                        let body_lock = bodies_singleton().lock().unwrap();
                        if let Some(body) = body_lock.collision_objects.get(&pObject2) {
                            rid2 = body.get_base().get_rid();
                            instance_id2 = body.get_base().get_instance_id();
                            type2 = body.get_base().get_type();
                        }
                    }
                } else {
                    let body_lock = bodies_singleton().lock().unwrap();
                    if let Some(body) = body_lock.collision_objects.get(&pObject1) {
                        rid1 = body.get_base().get_rid();
                        instance_id1 = body.get_base().get_instance_id();
                        type1 = body.get_base().get_type();
                    }
                    if let Some(body) = body_lock.collision_objects.get(&pObject2) {
                        rid2 = body.get_base().get_rid();
                        instance_id2 = body.get_base().get_instance_id();
                        type2 = body.get_base().get_type();
                    }
                }

        if event_info.is_sensor {
            if instance_id1 == 0 {
                godot_error!("Should be able to get info about a removed object if the other one is still valid.");
                return;
            }
            if instance_id2 == 0 {
                godot_error!( "Should be able to get info about a removed object if the other one is still valid.");
                return;
            }
    
            if type1 != CollisionObjectType::Area {
                if type2 != CollisionObjectType::Area {
                    godot_error!("Expected Area.");
                    return;
                }
                swap(&mut pObject1, &mut pObject2);
                swap(&mut type1, &mut type2);
                swap(&mut shape1, &mut shape2);
                swap(&mut collider_handle1, &mut collider_handle2);
                swap(&mut rid1, &mut rid2);
                swap(&mut instance_id1, &mut instance_id2);
            }
            
            let mut body_lock = bodies_singleton().lock().unwrap();
            let mut p_area = body_lock.collision_objects.get_mut(&pObject1).unwrap().get_mut_area();
            if type2 == CollisionObjectType::Area {
                let p_area2 = body_lock.collision_objects.get_mut(&pObject2).unwrap().get_mut_area();
                if event_info.is_started {
                    let mut p_area = p_area.unwrap();
                    p_area.on_area_enter(
                        collider_handle2,
                        p_area2,
                        shape2,
                        rid2,
                        instance_id2,
                        collider_handle1,
                        shape1,
                    );
                    p_area2.unwrap().on_area_enter(
                        collider_handle1,
                        Some(p_area),
                        shape1,
                        rid1,
                        instance_id1,
                        collider_handle2,
                        shape2,
                    );
                } else if event_info.is_stopped {
                    if let Some(pArea) = p_area {
                        pArea.on_area_exit(
                            collider_handle2,
                            p_area2,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                        );
                    } else {
                        // Try to retrieve area if not destroyed yet
                        let p_area = body_lock.collision_objects.get(&rid1).unwrap().get_mut_area();
                        if let Some(p_area) = p_area {
                            // Use invalid area case to keep counters consistent for already removed collider
                            p_area.on_area_exit(
                                collider_handle2,
                                None,
                                shape2,
                                rid2,
                                instance_id2,
                                collider_handle1,
                                shape1,
                            );
                        }
                    }
                    if let Some(p_area_2) = p_area2 {
                        p_area_2.on_area_exit(
                            collider_handle1,
                            p_area,
                            shape1,
                            rid1,
                            instance_id1,
                            collider_handle2,
                            shape2,
                        );
                    } else {
                        // Try to retrieve area if not destroyed yet
                        let p_area2 = body_lock.collision_objects.get_mut(&rid2).unwrap().get_mut_area();
                        if let Some(p_area_2) = p_area2 {
                            // Use invalid area case to keep counters consistent for already removed collider
                            p_area_2.on_area_exit(
                                collider_handle1,
                                None,
                                shape1,
                                rid1,
                                instance_id1,
                                collider_handle2,
                                shape2,
                            );
                        }
                    }
                }
            } else {
                let p_body = body_lock.collision_objects.get_mut(&pObject2).unwrap().get_mut_body();
                if event_info.is_started {
                    if !p_area.is_some() {
                        godot_error!("Should be able to get info about a removed object if the other one is still valid.");
                        return;
                    }
                    p_area.unwrap().on_body_enter(
                        collider_handle2,
                        p_body,
                        shape2,
                        rid2,
                        instance_id2,
                        collider_handle1,
                        shape1,
                    );
                } else if let Some(p_area) = p_area {
                    if event_info.is_stopped {
                        p_area.on_body_exit(
                            collider_handle2,
                            p_body,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                            true,
                        );
                    }
                } else if event_info.is_stopped {
                    // Try to retrieve area if not destroyed yet
                    let p_area = body_lock.collision_objects.get_mut(&rid1).unwrap().get_mut_area();
                    if let Some(p_area) = p_area {
                        // Use invalid body case to keep counters consistent for already removed collider
                        p_area.on_body_exit(
                            collider_handle2,
                            None,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                            false,
                        );
                    }
                }
            }
        } else {
            // Body contacts use contact_force_event_callback instead
            godot_error!("Shouldn't receive rigidbody collision events.");
        }
            }
        }
        */
    }

    pub fn contact_force_event_callback(
        world_handle: Handle,
        event_info: &ContactForceEventInfo,
    ) -> bool {
        let spaces_lock = spaces_singleton().lock().unwrap();
        let active_spaces_lock = active_spaces_singleton().lock().unwrap();
        let mut send_contacts = false;
        if let Some(space) = active_spaces_lock.active_spaces.get(&world_handle) {
            if let Some(space) = spaces_lock.spaces.get(space) {
                send_contacts = space.is_debugging_contacts();
            }

        let (pObject1,_)  = 
        RapierCollisionObject2D::get_collider_user_data(&event_info.user_data1);

        let (pObject2,_)  = 
        RapierCollisionObject2D::get_collider_user_data(&event_info.user_data2);
        let bodies_lock = bodies_singleton().lock().unwrap();
        if let Some(body1) = bodies_lock.collision_objects.get(&pObject1) {
            if let Some(body1) = body1.get_body() {
                if body1.can_report_contacts() {
                    send_contacts = true;
                }
            }
        }
        if let Some(body2) = bodies_lock.collision_objects.get(&pObject2) {
            if let Some(body2) = body2.get_body() {
                if body2.can_report_contacts() {
                    send_contacts = true;
                }
            }
        }

        return send_contacts;
    }
    false
    }

    pub fn contact_point_callback(
        world_handle: Handle,
        contact_info: &ContactPointInfo,
        event_info: &ContactForceEventInfo,
    ) -> bool {
        // Implement callback logic
        false
    }

}