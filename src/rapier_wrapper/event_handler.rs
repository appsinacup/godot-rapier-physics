use crossbeam::channel::Sender;
use godot::global::godot_error;
use rapier::crossbeam;
use rapier::prelude::*;
pub struct ContactEventHandler {
    collision_send: Sender<CollisionEvent>,
    contact_force_send: Sender<ContactPair>,
}
impl ContactEventHandler {
    pub fn new(
        collision_send: Sender<CollisionEvent>,
        contact_force_send: Sender<ContactPair>,
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
        _contact_pair: Option<&ContactPair>,
    ) {
        match self.collision_send.send(event) {
            Ok(_) => (),
            Err(err) => {
                godot_error!("Failed to send collision event {}", err.to_string());
            }
        }
    }

    fn handle_contact_force_event(
        &self,
        _dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        contact_pair: &ContactPair,
        _total_force_magnitude: Real,
    ) {
        match self.contact_force_send.send(contact_pair.clone()) {
            Ok(_) => (),
            Err(err) => {
                godot_error!("Failed to send contact force event {}", err.to_string());
            }
        }
    }
}
