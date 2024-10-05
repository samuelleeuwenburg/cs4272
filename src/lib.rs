#![no_std]

use core::cell::{Ref, RefCell};
use cortex_m::interrupt::{CriticalSection, Mutex};
use cortex_m::peripheral::NVIC;
use embedded_hal::digital::OutputPin;
use hal::dma::single_buffer;
use hal::gpio::{AnyPin, SpecificPin};
use hal::pio::{PIOExt, Running, Rx, StateMachine, Tx, PIO, SM0, SM1};
use rp235x_hal as hal;
use rp235x_hal::dma::single_buffer::Transfer;
use rp235x_hal::dma::SingleChannel;

const BIT_DEPTH: u32 = 16;
const CHANNELS: u32 = 2;
const PIO_STEPS: u32 = 4;

#[allow(dead_code)]
pub struct Cs4272<const BUFFER_SIZE: usize, Pio, ChA, ChB, P0, P1, P2, P3, P4, P5>
where
    Pio: PIOExt,
    ChA: SingleChannel,
    ChB: SingleChannel,
    P0: OutputPin,
    P1: AnyPin,
    P2: AnyPin,
    P3: AnyPin,
    P4: AnyPin,
    P5: AnyPin,
{
    buffers_ready: bool,
    buffer_tx: Mutex<RefCell<&'static mut [u32; BUFFER_SIZE]>>,
    buffer_rx: Mutex<RefCell<&'static mut [u32; BUFFER_SIZE]>>,
    transfer_tx: Option<Transfer<ChA, &'static mut [u32; BUFFER_SIZE], Tx<(Pio, SM1)>>>,
    transfer_rx: Option<Transfer<ChB, Rx<(Pio, SM1)>, &'static mut [u32; BUFFER_SIZE]>>,
    pio: PIO<Pio>,
    sm_mclk: StateMachine<(Pio, SM0), Running>,
    sm_i2s: StateMachine<(Pio, SM1), Running>,
    pin_enable: P0,
    pin_mclk: SpecificPin<P1>,
    pin_in: SpecificPin<P2>,
    pin_out: SpecificPin<P3>,
    pin_ws: SpecificPin<P4>,
    pin_clk: SpecificPin<P5>,
}

impl<const BUFFER_SIZE: usize, Pio, ChA, ChB, P0, P1, P2, P3, P4, P5>
    Cs4272<BUFFER_SIZE, Pio, ChA, ChB, P0, P1, P2, P3, P4, P5>
where
    Pio: PIOExt,
    ChA: SingleChannel,
    ChB: SingleChannel,
    P0: OutputPin,
    P1: AnyPin<Function = Pio::PinFunction>,
    P2: AnyPin<Function = Pio::PinFunction>,
    P3: AnyPin<Function = Pio::PinFunction>,
    P4: AnyPin<Function = Pio::PinFunction>,
    P5: AnyPin<Function = Pio::PinFunction>,
{
    pub fn new(
        pio: Pio,
        mut channel_a: ChA,
        mut channel_b: ChB,
        mut pin_enable: P0,
        pin_mclk: P1,
        pin_in: P2,
        pin_out: P3,
        pin_ws: P4,
        pin_clk: P5,
        buffer_tx_a: &'static mut [u32; BUFFER_SIZE],
        buffer_tx_b: &'static mut [u32; BUFFER_SIZE],
        buffer_rx_a: &'static mut [u32; BUFFER_SIZE],
        buffer_rx_b: &'static mut [u32; BUFFER_SIZE],
        resets: &mut hal::pac::RESETS,
        system_frequency: fugit::HertzU32,
        i2s_frequency: fugit::HertzU32,
    ) -> Self {
        let pin_mclk = pin_mclk.into();
        let pin_in = pin_in.into();
        let pin_out = pin_out.into();
        let pin_ws = pin_ws.into();
        let pin_clk = pin_clk.into();

        pin_enable.set_low().unwrap();

        let (mut pio, sm_mclk, sm_i2s, _, _) = pio.split(resets);

        let pio_mclk = pio_proc::pio_file!("src/i2s.pio", select_program("mclk"));
        let pio_i2s = pio_proc::pio_file!("src/i2s.pio", select_program("i2s"));

        let installed_mclk = pio.install(&pio_mclk.program).unwrap();
        let installed_i2s = pio.install(&pio_i2s.program).unwrap();

        let i2s_clk_frequency = i2s_frequency.raw() * BIT_DEPTH * CHANNELS * PIO_STEPS;

        let (i2s_int, i2s_frac) =
            get_divisions_for_frequency(system_frequency.raw(), i2s_clk_frequency);

        // According to the datasheet MCLK should be set to 8x the CLK speed
        let (mclk_int, mclk_frac) =
            get_divisions_for_frequency(system_frequency.raw(), i2s_clk_frequency * 8);

        let (mut sm_mclk, _, _) = hal::pio::PIOBuilder::from_installed_program(installed_mclk)
            .set_pins(pin_mclk.id().num, 1)
            .clock_divisor_fixed_point(mclk_int, mclk_frac)
            .build(sm_mclk);

        let (mut sm_i2s, rx_i2s, tx_i2s) =
            hal::pio::PIOBuilder::from_installed_program(installed_i2s)
                .in_pin_base(pin_in.id().num)
                .out_pins(pin_out.id().num, 1)
                .side_set_pin_base(pin_ws.id().num)
                .clock_divisor_fixed_point(i2s_int, i2s_frac)
                .autopull(true)
                .pull_threshold(32)
                .autopush(true)
                .push_threshold(32)
                .in_shift_direction(rp235x_hal::pio::ShiftDirection::Left)
                .out_shift_direction(rp235x_hal::pio::ShiftDirection::Left)
                .build(sm_i2s);

        sm_mclk.set_pindirs([(pin_mclk.id().num, hal::pio::PinDir::Output)]);

        sm_i2s.set_pindirs([
            (pin_in.id().num, hal::pio::PinDir::Input),
            (pin_out.id().num, hal::pio::PinDir::Output),
            (pin_ws.id().num, hal::pio::PinDir::Output),
            (pin_clk.id().num, hal::pio::PinDir::Output),
        ]);

        // Start the PIO state machines
        let sm_mclk = sm_mclk.start();
        let sm_i2s = sm_i2s.start();

        // Enable the interrupts
        unsafe {
            NVIC::unmask(hal::pac::Interrupt::DMA_IRQ_0);
            NVIC::unmask(hal::pac::Interrupt::DMA_IRQ_1);
        }

        channel_a.enable_irq0();
        channel_b.enable_irq1();

        // Start the DMA transfers
        let transfer_tx = single_buffer::Config::new(channel_a, buffer_tx_a, tx_i2s).start();
        let transfer_rx = single_buffer::Config::new(channel_b, rx_i2s, buffer_rx_a).start();

        // Pull up the enable pin
        pin_enable.set_high().unwrap();

        Cs4272 {
            pio,
            sm_mclk,
            sm_i2s,
            pin_enable,
            pin_mclk,
            pin_in,
            pin_out,
            pin_ws,
            pin_clk,
            buffer_tx: Mutex::new(RefCell::new(buffer_tx_b)),
            buffer_rx: Mutex::new(RefCell::new(buffer_rx_b)),
            transfer_tx: Some(transfer_tx),
            transfer_rx: Some(transfer_rx),
            buffers_ready: false,
        }
    }

    pub fn handle_irq0(&mut self, cs: &CriticalSection) {
        let mut transfer = self.transfer_tx.take().unwrap();
        transfer.check_irq0();

        let (channel, previous_buffer, tx) = transfer.wait();

        // Swap the buffers
        let next_buffer = self.buffer_tx.borrow(cs).replace(previous_buffer);

        // Restart the DMA transfer
        self.transfer_tx = Some(single_buffer::Config::new(channel, next_buffer, tx).start());
        self.buffers_ready = true;
    }

    pub fn handle_irq1(&mut self, cs: &CriticalSection) {
        let mut transfer = self.transfer_rx.take().unwrap();
        transfer.check_irq1();

        let (channel, rx, previous_buffer) = transfer.wait();

        // Swap the buffers
        let next_buffer = self.buffer_rx.borrow(cs).replace(previous_buffer);

        // Restart the DMA transfer
        self.transfer_rx = Some(single_buffer::Config::new(channel, rx, next_buffer).start());
    }

    pub fn poll(&self) -> bool {
        self.buffers_ready
    }

    pub fn get_input_buffer<'a>(
        &'a self,
        cs: &'a CriticalSection,
    ) -> Ref<'a, &'static mut [u32; BUFFER_SIZE]> {
        self.buffer_rx.borrow(cs).borrow()
    }

    pub fn set_output_buffer(&mut self, cs: &CriticalSection, buffer: &[u32; BUFFER_SIZE]) {
        let mut buffer_tx = self.buffer_tx.borrow(cs).borrow_mut();
        buffer_tx.copy_from_slice(buffer);
        self.buffers_ready = false;
    }
}

fn get_divisions_for_frequency(cpu: u32, target: u32) -> (u16, u8) {
    let int = cpu / target;
    let fraction = ((cpu % target) as f32 / target as f32) * 256.0;

    (int as u16, fraction as u8)
}
