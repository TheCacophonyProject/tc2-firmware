pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts(core::ptr::from_ref::<T>(p).cast::<u8>(), size_of::<T>()) }
}
