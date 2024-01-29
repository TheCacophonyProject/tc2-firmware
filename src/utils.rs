pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    core::slice::from_raw_parts((p as *const T) as *const u8, core::mem::size_of::<T>())
}
pub unsafe fn any_as_u8_slice_mut<T: Sized>(p: &mut T) -> &mut [u8] {
    core::slice::from_raw_parts_mut((p as *mut T) as *mut u8, core::mem::size_of::<T>())
}

pub unsafe fn u8_slice_to_u16(p: &[u8]) -> &[u16] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u16, p.len() / 2)
}

pub unsafe fn u16_slice_to_u8(p: &[u16]) -> &[u8] {
    core::slice::from_raw_parts((p as *const [u16]) as *const u8, p.len() * 2)
}

pub unsafe fn u16_slice_to_u8_mut(p: &mut [u16]) -> &mut [u8] {
    core::slice::from_raw_parts_mut((p as *mut [u16]) as *mut u8, p.len() * 2)
}

pub unsafe fn i32_slice_to_u8(p: &[i32]) -> &[u8] {
    core::slice::from_raw_parts((p as *const [i32]) as *const u8, p.len() * 4)
}

pub unsafe fn u8_slice_to_u32(p: &[u8]) -> &[u32] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u32, p.len() / 4)
}

pub unsafe fn any_as_u32_slice<T: Sized>(p: &T) -> &'static [u32] {
    core::slice::from_raw_parts((p as *const T) as *const u32, core::mem::size_of::<T>() / 4)
}
