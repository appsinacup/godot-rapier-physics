#[derive(PartialEq, Eq, Hash, Clone, Copy)]
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct RapierID {
    pub id: i32,
}
impl RapierID {
    pub fn new() -> Self {
        Self { id: rand::random() }
    }
}
