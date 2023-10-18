use crate::FrameSeg;
use core::cell::RefCell;
use critical_section::Mutex;

// pub static FRAME_SEGMENT_BUFFER: DoubleBuffer = DoubleBuffer {
//     front: Mutex::new(RefCell::new([
//         FrameSeg::new(),
//         FrameSeg::new(),
//         FrameSeg::new(),
//         FrameSeg::new(),
//     ])),
//     back: Mutex::new(RefCell::new([
//         FrameSeg::new(),
//         FrameSeg::new(),
//         FrameSeg::new(),
//         FrameSeg::new(),
//     ])),
//     swapper: Mutex::new(RefCell::new(true)),
// };

pub struct DoubleBuffer {
    front: Mutex<RefCell<[FrameSeg; 4]>>,
    back: Mutex<RefCell<[FrameSeg; 4]>>,
    swapper: Mutex<RefCell<bool>>,
}

impl DoubleBuffer {
    pub fn swap(&self) {
        critical_section::with(|cs| {
            let mut val = self.swapper.borrow_ref_mut(cs);
            *val = !*val;
        });
    }

    pub fn get_front(&self) -> &Mutex<RefCell<[FrameSeg; 4]>> {
        let mut val = false;
        critical_section::with(|cs| {
            val = *self.swapper.borrow(cs).borrow();
        });
        if val {
            &self.front
        } else {
            &self.back
        }
    }

    pub fn get_back(&self) -> &Mutex<RefCell<[FrameSeg; 4]>> {
        let mut val = false;
        critical_section::with(|cs| {
            val = *self.swapper.borrow(cs).borrow();
        });
        if val {
            &self.back
        } else {
            &self.front
        }
    }
}
