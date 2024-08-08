use std::mem::swap;

use godot::prelude::*;
use servers::rapier_physics_singleton::PhysicsCollisionObjects;

use super::rapier_space::RapierSpace;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::types::*;
use crate::*;
pub struct CollidersInfo {
    pub shape1: usize,
    pub object1: Rid,
    pub shape2: usize,
    pub object2: Rid,
}
impl Default for CollidersInfo {
    fn default() -> Self {
        Self {
            shape1: 0,
            object1: Rid::Invalid,
            shape2: 0,
            object2: Rid::Invalid,
        }
    }
}
impl RapierSpace {
    pub fn before_active_body_callback(
        &mut self,
        active_body_info: &BeforeActiveBodyInfo,
        physics_collision_objects: &mut PhysicsCollisionObjects,
    ) {
        let (rid, _) =
            RapierCollisionObject::get_collider_user_data(&active_body_info.body_user_data);
        if let Some(body) = physics_collision_objects.get_mut(&rid)
            && let Some(body) = body.get_mut_body()
        {
            body.set_previous_linear_velocity(vector_to_godot(active_body_info.previous_velocity));
        }
    }

    pub fn active_body_callback(
        &mut self,
        active_body_info: &ActiveBodyInfo,
        physics_collision_objects: &mut PhysicsCollisionObjects,
    ) {
        let (rid, _) =
            RapierCollisionObject::get_collider_user_data(&active_body_info.body_user_data);
        if let Some(body) = physics_collision_objects.get_mut(&rid)
            && let Some(body) = body.get_mut_body()
        {
            body.on_marked_active(self);
        }
    }

    pub fn collision_filter_body_callback(
        filter_info: &CollisionFilterInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        let mut colliders_info = CollidersInfo::default();
        (colliders_info.object1, colliders_info.shape1) =
            RapierCollisionObject::get_collider_user_data(&filter_info.user_data1);
        (colliders_info.object2, colliders_info.shape2) =
            RapierCollisionObject::get_collider_user_data(&filter_info.user_data2);
        if let Some(body1) = physics_collision_objects.get(&colliders_info.object1)
            && let Some(body1) = body1.get_body()
            && let Some(body2) = physics_collision_objects.get(&colliders_info.object2)
            && let Some(body2) = body2.get_body()
            && (body1.has_exception(body2.get_base().get_rid())
                || body2.has_exception(body1.get_base().get_rid()))
        {
            return false;
        }
        true
    }

    pub fn collision_modify_contacts_callback(
        filter_info: &CollisionFilterInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> OneWayDirection {
        let mut result = OneWayDirection::default();
        let (object1, shape1) =
            RapierCollisionObject::get_collider_user_data(&filter_info.user_data1);
        let (object2, shape2) =
            RapierCollisionObject::get_collider_user_data(&filter_info.user_data2);
        if let Some(collision_object_1) = physics_collision_objects.get(&object1)
            && let Some(collision_object_2) = physics_collision_objects.get(&object2)
        {
            let collision_base_1 = collision_object_1.get_base();
            let collision_base_2 = collision_object_2.get_base();
            if !collision_base_1.is_shape_disabled(shape1)
                && !collision_base_2.is_shape_disabled(shape2)
            {
                result.body1 = collision_base_1.is_shape_set_as_one_way_collision(shape1);
                result.pixel_body1_margin =
                    collision_base_1.get_shape_one_way_collision_margin(shape1);
                if let Some(body1) = collision_object_1.get_body() {
                    result.previous_linear_velocity1 =
                        vector_to_rapier(body1.get_previous_linear_velocity());
                }
                result.body2 = collision_base_2.is_shape_set_as_one_way_collision(shape2);
                result.pixel_body2_margin =
                    collision_base_2.get_shape_one_way_collision_margin(shape2);
                if let Some(body2) = collision_object_2.get_body() {
                    result.previous_linear_velocity2 =
                        vector_to_rapier(body2.get_previous_linear_velocity());
                }
            }
        }
        result
    }

    pub fn collision_event_callback(
        &mut self,
        event_info: &CollisionEventInfo,
        physics_collision_objects: &mut PhysicsCollisionObjects,
    ) {
        let (mut p_object1, mut shape1) =
            RapierCollisionObject::get_collider_user_data(&event_info.user_data1);
        let (mut p_object2, mut shape2) =
            RapierCollisionObject::get_collider_user_data(&event_info.user_data2);
        let mut collider_handle1 = event_info.collider1;
        let mut collider_handle2 = event_info.collider2;
        let (mut rid1, mut rid2) = (Rid::Invalid, Rid::Invalid);
        let (mut instance_id1, mut instance_id2) = (0, 0);
        let (mut type1, mut type2) = (CollisionObjectType::Area, CollisionObjectType::Area);
        if let Some(removed_collider_info_1) = self.get_removed_collider_info(&collider_handle1) {
            rid1 = removed_collider_info_1.rid;
            p_object1 = rid1;
            instance_id1 = removed_collider_info_1.instance_id;
            type1 = removed_collider_info_1.collision_object_type;
            shape1 = removed_collider_info_1.shape_index;
        } else if let Some(body) = physics_collision_objects.get(&p_object1) {
            rid1 = body.get_base().get_rid();
            instance_id1 = body.get_base().get_instance_id();
            type1 = body.get_base().get_type();
        }
        if let Some(removed_collider_info_2) = self.get_removed_collider_info(&collider_handle2) {
            rid2 = removed_collider_info_2.rid;
            p_object2 = rid2;
            instance_id2 = removed_collider_info_2.instance_id;
            type2 = removed_collider_info_2.collision_object_type;
            shape2 = removed_collider_info_2.shape_index;
        } else if let Some(body) = physics_collision_objects.get(&p_object2) {
            rid2 = body.get_base().get_rid();
            instance_id2 = body.get_base().get_instance_id();
            type2 = body.get_base().get_type();
        }
        if event_info.is_sensor {
            if rid1.is_invalid() || rid2.is_invalid() {
                godot_error!("Should be able to get info about a removed object if the other one is still valid.");
                return;
            }
            if type1 != CollisionObjectType::Area {
                if type2 != CollisionObjectType::Area {
                    godot_error!("Expected Area.");
                    return;
                }
                swap(&mut p_object1, &mut p_object2);
                swap(&mut type1, &mut type2);
                swap(&mut shape1, &mut shape2);
                swap(&mut collider_handle1, &mut collider_handle2);
                swap(&mut rid1, &mut rid2);
                swap(&mut instance_id1, &mut instance_id2);
            }
            let mut p_collision_object1 = None;
            let mut p_collision_object2 = None;
            if physics_collision_objects.contains_key(&p_object1)
                && physics_collision_objects.contains_key(&p_object2)
            {
                if let Some([p_object1, p_object2]) =
                    physics_collision_objects.get_many_mut([&p_object1, &p_object2])
                {
                    p_collision_object1 = Some(p_object1);
                    p_collision_object2 = Some(p_object2);
                }
            } else if physics_collision_objects.contains_key(&p_object1) {
                if let Some(p_object1) = physics_collision_objects.get_mut(&p_object1) {
                    p_collision_object1 = Some(p_object1);
                }
            } else if let Some(p_object2) = physics_collision_objects.get_mut(&p_object2) {
                p_collision_object2 = Some(p_object2);
            }
            // collision object 1 area
            if let Some(ref mut p_collision_object1) = p_collision_object1 {
                if let Some(p_area1) = p_collision_object1.get_mut_area() {
                    if type2 == CollisionObjectType::Area {
                        if event_info.is_started {
                            p_area1.on_area_enter(
                                collider_handle2,
                                &mut p_collision_object2,
                                shape2,
                                rid2,
                                instance_id2,
                                collider_handle1,
                                shape1,
                                self,
                            );
                        } else if event_info.is_stopped {
                            p_area1.on_area_exit(
                                collider_handle2,
                                &mut p_collision_object2,
                                shape2,
                                rid2,
                                instance_id2,
                                collider_handle1,
                                shape1,
                                self,
                            );
                        }
                    } else if event_info.is_started {
                        p_area1.on_body_enter(
                            collider_handle2,
                            &mut p_collision_object2,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                            self,
                        );
                    } else if event_info.is_stopped {
                        p_area1.on_body_exit(
                            collider_handle2,
                            &mut p_collision_object2,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                            self,
                        );
                    }
                }
            }
            // collision object 2 area
            if let Some(ref mut p_collision_object2) = p_collision_object2 {
                if let Some(p_area2) = p_collision_object2.get_mut_area() {
                    if type1 == CollisionObjectType::Area {
                        if event_info.is_started {
                            p_area2.on_area_enter(
                                collider_handle1,
                                &mut p_collision_object1,
                                shape1,
                                rid1,
                                instance_id1,
                                collider_handle2,
                                shape2,
                                self,
                            );
                        }
                        if event_info.is_stopped {
                            p_area2.on_area_exit(
                                collider_handle1,
                                &mut p_collision_object1,
                                shape1,
                                rid1,
                                instance_id1,
                                collider_handle2,
                                shape2,
                                self,
                            );
                        }
                    } else {
                        if event_info.is_started {
                            p_area2.on_body_enter(
                                collider_handle1,
                                &mut p_collision_object1,
                                shape1,
                                rid1,
                                instance_id1,
                                collider_handle2,
                                shape2,
                                self,
                            );
                        }
                        if event_info.is_stopped {
                            p_area2.on_body_exit(
                                collider_handle1,
                                &mut p_collision_object1,
                                shape1,
                                rid1,
                                instance_id1,
                                collider_handle2,
                                shape2,
                                self,
                            );
                        }
                    }
                }
            }
        } else {
            // Body contacts use contact_force_event_callback instead
            godot_error!("Shouldn't receive rigidbody collision events.");
        }
    }

    pub fn contact_force_event_callback(
        &mut self,
        event_info: &ContactForceEventInfo,
        physics_collision_objects: &mut PhysicsCollisionObjects,
    ) -> bool {
        let mut send_contacts = self.is_debugging_contacts();
        let (p_object1, _) = RapierCollisionObject::get_collider_user_data(&event_info.user_data1);
        let (p_object2, _) = RapierCollisionObject::get_collider_user_data(&event_info.user_data2);
        if let Some(body1) = physics_collision_objects.get(&p_object1) {
            if let Some(body1) = body1.get_body() {
                if body1.can_report_contacts() {
                    send_contacts = true;
                }
            }
        }
        if let Some(body2) = physics_collision_objects.get(&p_object2) {
            if let Some(body2) = body2.get_body() {
                if body2.can_report_contacts() {
                    send_contacts = true;
                }
            }
        }
        send_contacts
    }

    pub fn contact_point_callback(
        &mut self,
        contact_info: &ContactPointInfo,
        event_info: &ContactForceEventInfo,
        physics_collision_objects: &mut PhysicsCollisionObjects,
    ) {
        let pos1 = contact_info.pixel_local_pos_1;
        let pos2 = contact_info.pixel_local_pos_2;
        if self.is_debugging_contacts() {
            self.add_debug_contact(vector_to_godot(pos1));
            self.add_debug_contact(vector_to_godot(pos2));
        }
        let (p_object1, shape1) =
            RapierCollisionObject::get_collider_user_data(&event_info.user_data1);
        let (p_object2, shape2) =
            RapierCollisionObject::get_collider_user_data(&event_info.user_data2);
        if let Some([p_object1, p_object2]) =
            physics_collision_objects.get_many_mut([&p_object1, &p_object2])
        {
            let depth = real::max(0.0, -contact_info.pixel_distance); // negative distance means penetration
            let normal = contact_info.normal;
            // send just impulse directly along normal
            let impulse = contact_info.pixel_impulse * vector_to_godot(normal);
            let vel_pos1 = contact_info.pixel_velocity_pos_1;
            let vel_pos2 = contact_info.pixel_velocity_pos_2;
            if let Some(body1) = p_object1.get_mut_body() {
                if let Some(body2) = p_object2.get_mut_body() {
                    let static_linvelocity_1 = body1.get_static_linear_velocity();
                    let static_linvelocity_2 = body2.get_static_linear_velocity();
                    let static_angvelocity_1 = body1.get_static_angular_velocity();
                    let static_angvelocity_2 = body2.get_static_angular_velocity();
                    if static_linvelocity_1 != Vector::ZERO {
                        body2.to_add_static_constant_linear_velocity(static_linvelocity_1);
                    }
                    if static_linvelocity_2 != Vector::ZERO {
                        body1.to_add_static_constant_linear_velocity(static_linvelocity_2);
                    }
                    if static_angvelocity_1 != ANGLE_ZERO {
                        body2.to_add_static_constant_angular_velocity(static_angvelocity_1);
                    }
                    if static_angvelocity_2 != ANGLE_ZERO {
                        body1.to_add_static_constant_angular_velocity(static_angvelocity_2);
                    }
                    if body1.can_report_contacts() {
                        let instance_id2 = body2.get_base().get_instance_id();
                        body1.add_contact(
                            vector_to_godot(pos1),
                            vector_to_godot(-normal),
                            depth,
                            shape1 as i32,
                            vector_to_godot(vel_pos1),
                            vector_to_godot(pos2),
                            shape2 as i32,
                            instance_id2,
                            body2.get_base().get_rid(),
                            vector_to_godot(vel_pos2),
                            impulse,
                        );
                    }
                    if body2.can_report_contacts() {
                        let instance_id1 = body1.get_base().get_instance_id();
                        body2.add_contact(
                            vector_to_godot(pos2),
                            vector_to_godot(normal),
                            depth,
                            shape2 as i32,
                            vector_to_godot(vel_pos2),
                            vector_to_godot(pos1),
                            shape1 as i32,
                            instance_id1,
                            body1.get_base().get_rid(),
                            vector_to_godot(vel_pos1),
                            -impulse,
                        );
                    }
                }
            }
        }
    }
}
