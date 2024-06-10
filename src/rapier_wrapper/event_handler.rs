use crossbeam::channel::Sender;
use godot::log::godot_error;
use rapier::crossbeam;
use rapier::prelude::*;
pub struct ContactEventHandler {
    collision_send: Sender<(CollisionEvent, Option<ContactPair>)>,
    contact_force_send: Sender<(ContactForceEvent, ContactPair)>,
}
impl ContactEventHandler {
    pub fn new(
        collision_send: Sender<(CollisionEvent, Option<ContactPair>)>,
        contact_force_send: Sender<(ContactForceEvent, ContactPair)>,
    ) -> Self {
        ContactEventHandler {
            collision_send,
            contact_force_send,
        }
    }
}
impl EventHandler for ContactEventHandler {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        contact_pair: Option<&ContactPair>,
    ) {
        match self.collision_send.send((event, contact_pair.cloned())) {
            Ok(_) => (),
            Err(err) => {
                godot_error!("Failed to send collision event {}", err.to_string());
            }
        }
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let result = ContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        match self.contact_force_send.send((result, contact_pair.clone())) {
            Ok(_) => (),
            Err(err) => {
                godot_error!("Failed to send contact force event {}", err.to_string());
            }
        }
    }
}
