#[repr(C)]
#[derive(Copy, Clone, Eq, Hash, PartialEq)]
pub struct UserData {
    pub part1 : u64,
    pub part2 : u64,
}

impl UserData {
    pub fn new(data : u128) -> UserData {
		let data2 : u128 = data >> 64;
		let data1 : u128 = data - (data2 << 64);
        UserData {
			part1 : data1.try_into().unwrap(),
			part2 : data2.try_into().unwrap(),
        }
    }

    pub fn is_valid(&self) -> bool {
        return (self.part1 != u64::MAX) && (self.part2 != u64::MAX);
    }

    pub fn get_data(&self) -> u128 {
		let data1 : u128 = self.part1.into();
		let data2 : u128 = self.part2.into();
        return data1 + (data2 << 64);
    }
}

#[no_mangle]
pub extern "C" fn invalid_user_data() -> UserData {
    UserData {
        part1 : u64::MAX,
        part2 : u64::MAX,
    }
}

#[no_mangle]
pub extern "C" fn is_user_data_valid(user_data : UserData) -> bool {
    return user_data.is_valid();
}
