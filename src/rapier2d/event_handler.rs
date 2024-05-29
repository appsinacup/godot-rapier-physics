use crossbeam::channel::Sender;
use rapier2d::crossbeam;
use rapier2d::prelude::*;

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
        self
            .collision_send
            .send((event, contact_pair.cloned()))
            .unwrap();
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
        self
            .contact_force_send
            .send((result, contact_pair.clone()))
            .unwrap();
    }
}
