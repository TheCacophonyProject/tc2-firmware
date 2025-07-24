use core::mem;
use cortex_m::asm::nop;
use fugit::ExtU32;
use rp2040_hal::Watchdog;

pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts(core::ptr::from_ref::<T>(p).cast::<u8>(), size_of::<T>()) }
}

pub unsafe fn extend_lifetime<'b>(r: &'b [u8]) -> &'static [u8] {
    unsafe { mem::transmute::<&'b [u8], &'static [u8]>(r) }
}

pub unsafe fn extend_lifetime_generic<'b, T>(r: &'b T) -> &'static T {
    unsafe { mem::transmute::<&'b T, &'static T>(r) }
}

pub unsafe fn extend_lifetime_generic_mut<'b, T>(r: &'b mut T) -> &'static mut T {
    unsafe { mem::transmute::<&'b mut T, &'static mut T>(r) }
}

pub unsafe fn extend_lifetime_mut<'b>(r: &'b mut [u8]) -> &'static mut [u8] {
    unsafe { mem::transmute::<&'b mut [u8], &'static mut [u8]>(r) }
}

pub fn restart(watchdog: &mut Watchdog) -> ! {
    // Is this long enough to print any message to the terminal about why we're restarting?
    watchdog.start(500.millis());
    loop {
        nop();
    }
}
