//! # CS4272 Crate
//!
//! I2S implementation for the
//! [Cirrus Logic CS4272](https://statics.cirrus.com/pubs/proDatasheet/CS4272_F2.pdf) IC
//! built using the PIO.
//!
//! This driver uses double buffering for both input and output: when one set of buffers is busy
//! reading/writing to and from the hardware the other set is available to be read and written to.
//!
//! The library requires you to handle the switching of the buffers using the `DMA_IRQ_0`
//! so it makes sense to put the driver into a `Mutex<RefCell<Option<Cs4272>>` for concurrent access.
//!
//! ```
//! const BUFFER_SIZE: usize = 32;
//!
//! static CS4272: Mutex<
//!     RefCell<
//!         Option<
//!             Cs4272<
//!                 BUFFER_SIZE,
//!                 PIO0,
//!                 Channel<CH0>,
//!                 Channel<CH1>,
//!                 Pin<Gpio0, FunctionSioOutput, PullDown>,
//!                 Pin<Gpio1, FunctionPio0, PullDown>,
//!                 Pin<Gpio2, FunctionPio0, PullDown>,
//!                 Pin<Gpio3, FunctionPio0, PullDown>,

//!                 Pin<Gpio5, FunctionPio0, PullDown>,
//!             >,
//!         >,
//!     >,
//! > = Mutex::new(RefCell::new(None));
//! ```
//!
//! And handle the interrupt:
//!
//! ```
//! #[interrupt]
//! fn DMA_IRQ_0() {
//!     cortex_m::interrupt::free(|cs| {
//!         if let Some(driver) = CS4272.borrow(cs).borrow_mut().as_mut() {
//!             driver.handle_irq0();
//!         }
//!     });
//! }
//! ```
//!
//! Using the [Cs4272::poll] method you can check wether or not the current available buffer has already
//! been written to. This makes it relatively easy to process the data whenever it makes sense for your
//! application:
//!
//! ```
//! cortex_m::interrupt::free(|cs| {
//!     if let Some(cs4272) = CS4272.borrow(cs).borrow_mut().as_mut() {
//!         // Check if the buffers are ready for processing
//!         if cs4272.poll() {
//!             // Make a copy of the input buffer for manipulation
//!             let mut buffer = cs4272.get_input_buffer().clone();
//!
//!             // Process input
//!             for _sample in buffer.iter_mut() { /* ... */ }
//!
//!             // Copy the processed buffer to the output
//!             cs4272.set_output_buffer(&input);
//!         }
//!     }
//! });
//! ```
//! Just make sure to keep the processing light or clone the data out of the `interrupt::free`
//! callback so you won't miss a buffer.
//!

#![no_std]

use core::cell::{Ref, RefCell};
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

/// Main driver struct, keeps ownership of all necessary resources
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
    buffer_rx: RefCell<&'static mut [u32; BUFFER_SIZE]>,
    buffer_tx: RefCell<&'static mut [u32; BUFFER_SIZE]>,
    buffers_ready: bool,
    pin_clk: SpecificPin<P5>,
    pin_enable: P0,
    pin_in: SpecificPin<P2>,
    pin_mclk: SpecificPin<P1>,
    pin_out: SpecificPin<P3>,
    pin_ws: SpecificPin<P4>,
    pio: PIO<Pio>,
    sm_i2s: StateMachine<(Pio, SM1), Running>,
    sm_mclk: StateMachine<(Pio, SM0), Running>,
    transfer_rx: Option<Transfer<ChB, Rx<(Pio, SM1)>, &'static mut [u32; BUFFER_SIZE]>>,
    transfer_tx: Option<Transfer<ChA, &'static mut [u32; BUFFER_SIZE], Tx<(Pio, SM1)>>>,
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
    /// Construct and start a new [`Cs4272`] instance.
    ///
    pub fn new(
        resets: &mut hal::pac::RESETS,
        mut pin_enable: P0,
        pin_mclk: P1,
        pin_in: P2,
        pin_out: P3,
        pin_ws: P4,
        pin_clk: P5,
        pio: Pio,
        mut channel_a: ChA,
        channel_b: ChB,
        buffer_tx_a: &'static mut [u32; BUFFER_SIZE],
        buffer_tx_b: &'static mut [u32; BUFFER_SIZE],
        buffer_rx_a: &'static mut [u32; BUFFER_SIZE],
        buffer_rx_b: &'static mut [u32; BUFFER_SIZE],
        system_frequency: fugit::HertzU32,
        sample_rate: fugit::HertzU32,
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

        let i2s_steps_per_cycle = 4;
        let mclk_steps_per_cycle = 2;
        let i2s_clk_frequency = sample_rate.raw() * BIT_DEPTH * CHANNELS;
        // According to the datasheet the mclk should be 8 times the clk frequency
        let i2s_mclk_frequency = i2s_clk_frequency * 8;

        // @TODO:
        // let (i2s_int, i2s_frac) = (24, 0);
        let (i2s_int, i2s_frac) = get_divisions_for_frequency(
            system_frequency.raw(),
            i2s_clk_frequency * i2s_steps_per_cycle,
        );

        // @TODO:
        // let (mclk_int, mclk_frac) = (6, 0);
        let (mclk_int, mclk_frac) = get_divisions_for_frequency(
            system_frequency.raw(),
            i2s_mclk_frequency * mclk_steps_per_cycle,
        );

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
                .pull_threshold(16)
                .autopush(true)
                .push_threshold(16)
                .in_shift_direction(rp235x_hal::pio::ShiftDirection::Right)
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

        // Enable the interrupt
        unsafe {
            NVIC::unmask(hal::pac::Interrupt::DMA_IRQ_0);
        }

        channel_a.enable_irq0();

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
            buffer_tx: RefCell::new(buffer_tx_b),
            buffer_rx: RefCell::new(buffer_rx_b),
            transfer_tx: Some(transfer_tx),
            transfer_rx: Some(transfer_rx),
            buffers_ready: false,
        }
    }

    /// Required method for restarting the DMA and swapping the buffers internally.
    ///
    /// ```
    /// #[interrupt]
    /// fn DMA_IRQ_0() {
    ///     cortex_m::interrupt::free(|cs| {
    ///         let mut cs4272 = CS4272.borrow(cs).borrow_mut();
    ///         cs4272.as_mut().unwrap().handle_irq0();
    ///     });
    /// }
    /// ```
    pub fn handle_irq0(&mut self) {
        let mut transfer_tx = self.transfer_tx.take().unwrap();

        transfer_tx.check_irq0();

        let (channel_tx, previous_buffer_tx, tx) = transfer_tx.wait();
        let next_buffer_tx = self.buffer_tx.replace(previous_buffer_tx);
        self.transfer_tx = Some(single_buffer::Config::new(channel_tx, next_buffer_tx, tx).start());

        let transfer_rx = self.transfer_rx.take().unwrap();
        let (channel_rx, rx, previous_buffer_rx) = transfer_rx.wait();
        let next_buffer_rx = self.buffer_rx.replace(previous_buffer_rx);
        self.transfer_rx = Some(single_buffer::Config::new(channel_rx, rx, next_buffer_rx).start());

        self.buffers_ready = true;
    }

    /// Check if the current free output buffer has already been written to.
    ///
    /// This makes it easy to periodically check when the driver is ready for new input.
    ///
    /// ```
    /// if cs4272.poll() {
    ///     // buffer is ready for new input
    /// }
    /// ```
    pub fn poll(&self) -> bool {
        self.buffers_ready
    }

    /// Get a reference to the currently free input buffer.
    ///
    /// If you are using the driver from a mutex getting to the data
    /// requires a `cs` from an `interrupt::free` callback for memory safety.
    /// Consider cloning the buffer to process it without blocking any other interrupts:
    ///
    /// ```
    /// let buffer = cortex_m::interrupt::free(|cs| {
    ///     CS4272.borrow(cs).borrow_mut().as_ref().unwrap().get_input_buffer().clone()
    /// });
    /// ```
    pub fn get_input_buffer<'a>(&'a self) -> Ref<'a, &'static mut [u32; BUFFER_SIZE]> {
        self.buffer_rx.borrow()
    }

    /// Sets the current free output buffer for the next buffer switch.
    ///
    /// ```
    /// // Output silence
    /// let buffer: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE];
    ///
    /// cortex_m::interrupt::free(|cs| {
    ///     CS4272.borrow(cs).borrow_mut().as_mut().unwrap().set_output_buffer(&buffer).clone()
    /// });
    /// ```
    pub fn set_output_buffer(&mut self, buffer: &[u32; BUFFER_SIZE]) {
        let mut buffer_tx = self.buffer_tx.borrow_mut();
        buffer_tx.copy_from_slice(buffer);
        self.buffers_ready = false;
    }
}

fn get_divisions_for_frequency(cpu: u32, target: u32) -> (u16, u8) {
    let int = cpu / target;
    let fraction = ((cpu % target) as f32 / target as f32) * 256.0;

    (int as u16, fraction as u8)
}
