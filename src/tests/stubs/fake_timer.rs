use fugit::TimerInstantU64;

pub type Instant = TimerInstantU64<1_000_000>;

#[derive(Copy, Clone)]
pub struct Timer {
    pub counter: u64,
}

impl Timer {
    pub fn delay_ms(&self, ms: u64) {
        //info!("Delaying for {}ms", ms);
    }
}

impl Timer {
    pub fn new() -> Timer {
        Timer { counter: 0 }
    }

    pub fn get_counter(&self) -> Instant {
        // TODO: Need to make counter increase?
        TimerInstantU64::from_ticks(self.counter)
    }

    pub fn delay_us(&mut self, micros: u32) {
        //info!("Delaying for {}µs", micros);
    }
}
