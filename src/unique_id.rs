use core::sync::atomic::Ordering;
use std::sync::atomic::AtomicUsize;

use godot::builtin::Rid;

use crate::servers::rapier_physics_singleton::physics_data;
pub fn new_uid() -> usize {
    static COUNTER: AtomicUsize = AtomicUsize::new(1);
    COUNTER.fetch_add(1, Ordering::Relaxed)
}
pub fn invalid_uid() -> usize {
    0
}
pub fn get_rid(uid: usize) -> &'static Rid {
    let physics_data = physics_data();
    if let Some(rid) = physics_data.rids.get(&uid) {
        return rid;
    }
    &Rid::Invalid
}
