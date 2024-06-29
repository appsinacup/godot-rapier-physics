use rapier::data::Index;
use salva::object::ContiguousArenaIndex;
use salva::object::FluidHandle;
#[derive(Debug, Copy, Clone, Eq, Hash, PartialEq)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct HandleDouble {
    id: usize,
    generation: u64,
}
impl Default for HandleDouble {
    fn default() -> HandleDouble {
        HandleDouble {
            id: usize::MAX,
            generation: u64::MAX,
        }
    }
}
pub type WorldHandle = Index;
pub type ShapeHandle = Index;
impl HandleDouble {
    pub fn is_valid(&self) -> bool {
        (self.id != usize::MAX) && (self.generation != u64::MAX)
    }
}
pub fn fluid_handle_to_handle(fluid_handle: FluidHandle) -> HandleDouble {
    let contiguous_index: ContiguousArenaIndex = fluid_handle.into();
    let raw_parts = contiguous_index.into_raw_parts();
    HandleDouble {
        id: raw_parts.0,
        generation: raw_parts.1,
    }
}
pub fn handle_to_fluid_handle(handle: HandleDouble) -> FluidHandle {
    FluidHandle::from(ContiguousArenaIndex::from_raw_parts(
        handle.id,
        handle.generation,
    ))
}
pub fn invalid_handle_double() -> HandleDouble {
    HandleDouble {
        id: usize::MAX,
        generation: u64::MAX,
    }
}
