use crate::bsp::pac::{DMA, RESETS, SPI1};
use byteorder::{ByteOrder, LittleEndian};
use crc::{Crc, CRC_16_XMODEM};
use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{
    _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
    _embedded_hal_spi_FullDuplex,
};
use rp2040_hal::dma::{single_buffer, Channel, CH0};
use rp2040_hal::gpio::bank0::{Gpio12, Gpio13, Gpio14, Gpio15, Gpio5};
use rp2040_hal::gpio::{
    FunctionNull, FunctionSio, FunctionSpi, Pin, PullDown, PullNone, SioOutput,
};
use rp2040_hal::Spi;

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum ExtTransferMessage {
    CameraConnectInfo = 0x1,
    CameraRawFrameTransfer = 0x2,
    BeginFileTransfer = 0x3,
    ResumeFileTransfer = 0x4,
    EndFileTransfer = 0x5,
    BeginAndEndFileTransfer = 0x6,
}

pub struct ExtSpiTransfers {
    pub spi: Option<
        Spi<
            rp2040_hal::spi::Enabled,
            SPI1,
            (
                Pin<Gpio15, FunctionSpi, PullNone>, // miso
                Pin<Gpio12, FunctionSpi, PullNone>, // mosi
                Pin<Gpio14, FunctionSpi, PullNone>, // clk
            ),
            8,
        >,
    >,
    cs_enabled: Option<Pin<Gpio13, FunctionSpi, PullNone>>, // cs
    miso_disabled: Option<Pin<Gpio15, FunctionNull, PullNone>>,
    mosi_disabled: Option<Pin<Gpio12, FunctionNull, PullNone>>,
    clk_disabled: Option<Pin<Gpio14, FunctionNull, PullNone>>,
    cs_disabled: Option<Pin<Gpio13, FunctionNull, PullNone>>,
    ping: Pin<Gpio5, FunctionSio<SioOutput>, PullDown>,
    dma_channel: Option<Channel<CH0>>,
    payload_buffer: Option<&'static mut [u8; 2066]>,
    crc_buffer: Option<&'static mut [u8; 32]>,
}

impl ExtSpiTransfers {
    pub fn new(
        mosi: Pin<Gpio12, FunctionNull, PullNone>,
        cs: Pin<Gpio13, FunctionNull, PullNone>,
        clk: Pin<Gpio14, FunctionNull, PullNone>,
        miso: Pin<Gpio15, FunctionNull, PullNone>,
        ping: Pin<Gpio5, FunctionSio<SioOutput>, PullDown>,
        dma_channel: Channel<CH0>,
        payload_buffer: &'static mut [u8; 2066],
        crc_buffer: &'static mut [u8; 32],
    ) -> ExtSpiTransfers {
        ExtSpiTransfers {
            spi: None,
            cs_enabled: None,
            mosi_disabled: Some(mosi),
            cs_disabled: Some(cs),
            clk_disabled: Some(clk),
            miso_disabled: Some(miso),
            ping,
            dma_channel: Some(dma_channel),
            payload_buffer: Some(payload_buffer),
            crc_buffer: Some(crc_buffer),
        }
    }

    pub fn enable(&mut self, spi: SPI1, resets: &mut RESETS) {
        self.cs_enabled = Some(
            self.cs_disabled
                .take()
                .unwrap()
                .into_function::<FunctionSpi>()
                .into_pull_type(),
        );
        let spi = Spi::<_, _, _, 8>::new(
            spi,
            (
                self.miso_disabled
                    .take()
                    .unwrap()
                    .into_function::<FunctionSpi>()
                    .into_pull_type(),
                self.mosi_disabled
                    .take()
                    .unwrap()
                    .into_function::<FunctionSpi>()
                    .into_pull_type(),
                self.clk_disabled
                    .take()
                    .unwrap()
                    .into_function::<FunctionSpi>()
                    .into_pull_type(),
            ),
        )
        .init_slave(resets, &embedded_hal::spi::MODE_3);
        self.spi = Some(spi);
        self.ping.set_low().unwrap();
    }

    pub fn send_message(
        &mut self,
        message_type: ExtTransferMessage,
        payload: &[u8],
        dma_peripheral: &mut DMA,
    ) {
        // The transfer header contains the transfer type (2x)
        // the number of bytes to read for the payload (should this be twice?)
        // the 16 bit crc of the payload (twice)

        // It is followed by the payload itself

        let mut transfer_header = [0u8; 1 + 1 + 4 + 4 + 2 + 2 + 2 + 2];
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);

        let crc = crc_check.checksum(&payload);
        transfer_header[0] = message_type as u8;
        transfer_header[1] = message_type as u8;
        LittleEndian::write_u32(&mut transfer_header[2..6], payload.len() as u32);
        LittleEndian::write_u32(&mut transfer_header[6..10], payload.len() as u32);
        LittleEndian::write_u16(&mut transfer_header[10..12], crc);
        LittleEndian::write_u16(&mut transfer_header[12..14], crc);
        LittleEndian::write_u16(&mut transfer_header[14..16], crc.reverse_bits());
        LittleEndian::write_u16(&mut transfer_header[16..=17], crc.reverse_bits());

        // info!("Writing header {}", &transfer_header);

        self.payload_buffer.as_mut().unwrap()[0..transfer_header.len()]
            .copy_from_slice(&transfer_header);
        self.payload_buffer.as_mut().unwrap()
            [transfer_header.len()..transfer_header.len() + payload.len()]
            .copy_from_slice(&payload);

        let mut transmit_success = false;
        while !transmit_success {
            self.ping.set_high().unwrap();
            {
                // Enable sniffing on the channel
                dma_peripheral.ch[0]
                    .ch_ctrl_trig
                    .write(|w| w.sniff_en().set_bit());
                // Set which channel the sniffer is sniffing
                dma_peripheral
                    .sniff_ctrl
                    .write(|w| unsafe { w.dmach().bits(0) });
                //let payload = self.payload_buffer.take().unwrap();
                {
                    let mut transfer = single_buffer::Config::new(
                        self.dma_channel.take().unwrap(),
                        self.payload_buffer.take().unwrap(),
                        self.spi.take().unwrap(),
                    )
                    .start();
                    self.ping.set_low().unwrap();
                    maybe_abort_dma_transfer(dma_peripheral, 0);
                    let (r_ch0, r_buf, spi) = transfer.wait();
                    self.dma_channel = Some(r_ch0);
                    self.payload_buffer = Some(r_buf);
                    self.spi = Some(spi);
                }
                //self.payload_buffer = Some(payload);
            }
            self.ping.set_high().unwrap();
            // Now read the crc from the pi
            {
                // TODO: Is all the DMA state reset now?

                let transfer = single_buffer::Config::new(
                    self.dma_channel.take().unwrap(),
                    self.spi.take().unwrap(),
                    self.crc_buffer.take().unwrap(),
                )
                .start();
                self.ping.set_low().unwrap();
                maybe_abort_dma_transfer(dma_peripheral, 0);
                let (r_ch0, spi, r_buf) = transfer.wait();
                self.dma_channel = Some(r_ch0);

                // Find offset crc in buffer:
                if let Some(start) = r_buf.iter().position(|&x| x == 1) {
                    if start < r_buf.len() - 8 {
                        let prelude = &r_buf[start + 1..start + 4];
                        if prelude[0] == 2 && prelude[1] == 3 && prelude[2] == 4 {
                            let crc_from_remote =
                                LittleEndian::read_u16(&r_buf[start + 4..start + 6]);
                            let crc_from_remote_dup =
                                LittleEndian::read_u16(&r_buf[start + 6..start + 8]);
                            if crc_from_remote == crc_from_remote_dup && crc_from_remote == crc {
                                //info!("Success!");
                                transmit_success = true;
                            }
                        }
                    }
                }
                self.crc_buffer = Some(r_buf);
                self.spi = Some(spi);
            }
        }
    }

    pub fn disable(&mut self) -> SPI1 {
        let spi = self.spi.take().unwrap();
        let spi_disabled = spi.disable();
        let (spi_free, (miso, mosi, clk)) = spi_disabled.free();
        self.mosi_disabled = Some(mosi.into_function::<FunctionNull>().into_pull_type());
        self.cs_disabled = Some(
            self.cs_enabled
                .take()
                .unwrap()
                .into_function::<FunctionNull>()
                .into_pull_type(),
        );
        self.clk_disabled = Some(clk.into_function::<FunctionNull>().into_pull_type());
        self.miso_disabled = Some(miso.into_function::<FunctionNull>().into_pull_type());
        spi_free
    }

    pub fn write(&mut self, bytes: &[u8]) {
        self.spi.as_mut().unwrap().write(bytes).unwrap()
    }

    pub fn flush(&mut self) {
        // while !self.is_empty() {
        //     let b = self.read_byte();
        // }
        //self.spi.as_mut().unwrap().flush().unwrap();

        //&mut hal::Spi<hal::spi::Enabled, SPI1, (hal::gpio::Pin<Gpio15, FunctionSpi, PullNone>, hal::gpio::Pin<Gpio12, FunctionSpi, PullNone>, hal::gpio::Pin<Gpio14, FunctionSpi, PullNone>), 8>

        //Option<hal::Spi<hal::spi::Enabled, SPI1, (hal::gpio::Pin<Gpio15, FunctionSpi, PullNone>, hal::gpio::Pin<Gpio12, FunctionSpi, PullNone>, hal::gpio::Pin<Gpio14, FunctionSpi, PullNone>), 8>>::flush()
    }

    pub fn schmitt(&self) -> bool {
        self.cs_enabled.as_ref().unwrap().get_schmitt_enabled()
    }

    pub fn align_sync(&mut self) {
        //info!("Aligning");
        //let pattern = [1, 2, 3, 4];
        let mut buf = [0u8; 1];

        // Master sends 255 continuously.
        // When we get it, we reply with 1

        // Master replies with 2
        // Master breaks, we break;

        loop {
            // Looks like transfer to master is always offset by one byte in this scheme, a write, then a read,
            // rather than shifting each out at the same time.
            let response = self.spi.as_mut().unwrap().transfer(&mut buf).unwrap();
            if response[0] == 255 {
                buf[0] = 1;
            } else if response[0] == 2 {
                break;
            }
        }
        //info!("Aligned");
    }

    pub fn transfer<'a>(&mut self, bytes: &'a mut [u8]) -> &'a [u8] {
        self.spi.as_mut().unwrap().transfer(bytes).unwrap()
    }

    pub fn is_busy(&self) -> bool {
        self.spi.as_ref().unwrap().is_busy()
    }
    pub fn is_empty(&self) -> bool {
        self.spi.as_ref().unwrap().is_empty()
    }

    pub fn read_byte(&mut self) -> u32 {
        if !self.is_empty() {
            self.spi.as_mut().unwrap().read().unwrap() as u32
        } else {
            1000u32
        }
    }
}

fn maybe_abort_dma_transfer(dma: &mut DMA, channel: u32) {
    // If the raspberry pi host requests less bytes than we want to send, we'd stall forever
    // unless we keep tabs on whether or not the channel has stalled, and abort appropriately.
    let mut prev_count = dma.ch[channel as usize].ch_trans_count.read().bits();
    let mut prev_crc = dma.sniff_data.read().bits();
    let mut same_count = 0;
    let mut same_crc = 0;
    let mut needs_abort = false;
    loop {
        let count = dma.ch[channel as usize].ch_trans_count.read().bits();
        let crc = dma.sniff_data.read().bits();
        if count == dma.ch0_dbg_tcr.read().bits() && crc != prev_crc {
            // Completed full transfer successfully.
            break;
        }
        if count == prev_count {
            same_count += 1;
        }
        if crc == prev_crc {
            same_crc += 1;
        }
        if same_count == 10_000 {
            needs_abort = true;
            break;
        }
        if same_crc == 10_000 {
            needs_abort = true;
            break;
        }
        prev_count = count;
        prev_crc = crc;
    }
    if needs_abort && dma.ch[0].ch_ctrl_trig.read().busy().bit_is_set() {
        info!(
            "Aborting dma transfer at count {} of {}, dreq {}",
            prev_count,
            dma.ch0_dbg_tcr.read().bits(),
            dma.ch0_dbg_ctdreq.read().bits()
        );
        // See RP2040-E13 in rp2040 datasheet for explanation of errata workaround.
        let inte0 = dma.inte0.read().bits();
        let inte1 = dma.inte1.read().bits();
        let mask = (1u32 << channel).reverse_bits();
        dma.inte0.write(|w| unsafe { w.bits(inte0 & mask) });
        dma.inte1.write(|w| unsafe { w.bits(inte1 & mask) });
        // Abort all dma transfers
        dma.chan_abort.write(|w| unsafe { w.bits(1 << channel) });

        while dma.ch[0].ch_ctrl_trig.read().busy().bit_is_set() {}

        dma.inte0.write(|w| unsafe { w.bits(inte0) });
        dma.inte1.write(|w| unsafe { w.bits(inte1) });
    }
}
