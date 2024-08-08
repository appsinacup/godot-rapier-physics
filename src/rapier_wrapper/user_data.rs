#[derive(Copy, Clone, Eq, Hash, PartialEq, Default, Debug)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct UserData {
    pub part1: u64,
    pub part2: u64,
}
impl UserData {
    pub fn new(data: u128) -> UserData {
        let data2 = data >> 64;
        let data1 = data - (data2 << 64);
        UserData {
            part1: data1 as u64,
            part2: data2 as u64,
        }
    }

    pub fn is_valid(&self) -> bool {
        (self.part1 != u64::MAX) && (self.part2 != u64::MAX)
    }

    pub fn get_data(&self) -> u128 {
        let data1: u128 = self.part1.into();
        let data2: u128 = self.part2.into();
        data1 + (data2 << 64)
    }

    pub fn invalid_user_data() -> UserData {
        UserData {
            part1: u64::MAX,
            part2: u64::MAX,
        }
    }
}
