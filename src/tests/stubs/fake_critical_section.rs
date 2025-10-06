// NOTE: Copied from critical_section crate

use core::marker::PhantomData;

#[derive(Clone, Copy, Debug)]
pub struct CriticalSection<'cs> {
    _private: PhantomData<&'cs ()>,
    _not_send_sync: PhantomData<*mut ()>,
}

impl<'cs> CriticalSection<'cs> {
    #[inline(always)]
    pub fn new() -> Self {
        CriticalSection {
            _private: PhantomData,
            _not_send_sync: PhantomData,
        }
    }
}

pub fn with<F, R>(f: F) -> R
where
    F: FnOnce(CriticalSection) -> R,
{
    f(CriticalSection::new())
}

use crate::frame_processing::FrameBuffer;
use crate::re_exports::bsp::hal::sio::copy_last_frame;
use core::cell::{Ref, RefCell, RefMut, UnsafeCell};

#[derive(Debug)]
pub struct Mutex<T> {
    inner: UnsafeCell<T>,
}

impl<T> Mutex<T> {
    #[inline]
    pub const fn new(value: T) -> Self {
        Mutex {
            inner: UnsafeCell::new(value),
        }
    }
    #[inline]
    pub fn get_mut(&mut self) -> &mut T {
        unsafe { &mut *self.inner.get() }
    }

    /// Unwraps the contained value, consuming the mutex.
    #[inline]
    pub fn into_inner(self) -> T {
        self.inner.into_inner()
    }

    /// Borrows the data for the duration of the critical section.
    #[inline]
    pub fn borrow<'cs>(&'cs self, _cs: CriticalSection<'cs>) -> &'cs T {
        unsafe { &*self.inner.get() }
    }
}

impl Mutex<RefCell<Option<&'static mut FrameBuffer>>> {
    #[inline]
    #[track_caller]
    pub fn borrow_ref_mut<'cs>(
        &'cs self,
        cs: CriticalSection<'cs>,
    ) -> RefMut<'cs, Option<&'static mut FrameBuffer>> {
        let mut fb = self.borrow(cs).borrow_mut();
        copy_last_frame(&mut fb);
        fb
    }
}

unsafe impl<T> Sync for Mutex<T> where T: Send {}
