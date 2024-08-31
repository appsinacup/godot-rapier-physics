use std::sync::atomic::AtomicUsize;
use core::sync::atomic::Ordering;

#[derive(PartialEq, Eq, Hash, Clone, Copy)]
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct RapierID {
    pub id: usize,
}
impl RapierID {
    pub fn new() -> Self {
        static COUNTER:AtomicUsize = AtomicUsize::new(1);
        Self { id: COUNTER.fetch_add(1, Ordering::Relaxed) }
    }
}
