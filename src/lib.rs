//! # CS4272 Crate
//!
//! I2S implementation for the
//! [Cirrus Logic CS4272](https://statics.cirrus.com/pubs/proDatasheet/CS4272_F2.pdf) IC
//! built using the PIO.
//!
//! The driver assumes the CS4272 is running in left-justified and slave mode.
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
//! Configure and start the driver:
//!
//! ```
//! let cs4272k = Cs4272Builder::new(
//!     pins.gpio0.into_push_pull_output(),
//!     pins.gpio1.into_function(),
//!     pins.gpio2.into_function(),
//!     pins.gpio3.into_function(),
//!     pins.gpio4.into_function(),
//!     pins.gpio5.into_function(),
//!     pac.PIO0,
//!     dma.ch0,
//!     dma.ch1,
//!     singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
//!     singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
//!     singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
//!     singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
//! )
//! .start(&mut pac.RESETS);
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
#![warn(missing_docs)]

use core::cell::{Ref, RefCell};
use cortex_m::peripheral::NVIC;
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use hal::dma::single_buffer;
use hal::dma::single_buffer::Transfer;
use hal::dma::SingleChannel;
use hal::gpio::{AnyPin, SpecificPin};
use hal::pio::{PIOExt, Running, Rx, StateMachine, Tx, PIO, SM0, SM1};
use rp235x_hal as hal;

enum DmaInterrupt {
    Irq0,
    Irq1,
}

/// [Cs4272] builder, used to initialize, configure and start the driver
pub struct Cs4272Builder<const BUFFER_SIZE: usize, Pio, ChA, ChB, P0, P1, P2, P3, P4, P5>
where
    P1: AnyPin,
    P2: AnyPin,
    P3: AnyPin,
    P4: AnyPin,
    P5: AnyPin,
{
    pio: Option<Pio>,
    channel_a: Option<ChA>,
    channel_b: Option<ChB>,
    buffer_tx_a: Option<&'static mut [u32; BUFFER_SIZE]>,
    buffer_tx_b: &'static mut [u32; BUFFER_SIZE],
    buffer_rx_a: Option<&'static mut [u32; BUFFER_SIZE]>,
    buffer_rx_b: &'static mut [u32; BUFFER_SIZE],
    pin_enable: P0,
    pin_mclk: SpecificPin<P1>,
    pin_in: SpecificPin<P2>,
    pin_out: SpecificPin<P3>,
    pin_ws: SpecificPin<P4>,
    pin_clk: SpecificPin<P5>,
    system_frequency: fugit::HertzU32,
    sample_rate: fugit::HertzU32,
    interrupt: DmaInterrupt,
}

impl<const BUFFER_SIZE: usize, Pio, ChA, ChB, P0, P1, P2, P3, P4, P5>
    Cs4272Builder<BUFFER_SIZE, Pio, ChA, ChB, P0, P1, P2, P3, P4, P5>
where
    P0: OutputPin,
    P1: AnyPin<Function = Pio::PinFunction>,
    P2: AnyPin<Function = Pio::PinFunction>,
    P3: AnyPin<Function = Pio::PinFunction>,
    P4: AnyPin<Function = Pio::PinFunction>,
    P5: AnyPin<Function = Pio::PinFunction>,
    Pio: PIOExt,
    ChA: SingleChannel,
    ChB: SingleChannel,
{
    /// Initialize a new builder.
    ///
    /// Requires passing all the necessary pins, DMAs, pio and buffers we need to run the driver.
    pub fn new(
        mut pin_enable: P0,
        pin_mclk: P1,
        pin_in: P2,
        pin_out: P3,
        pin_ws: P4,
        pin_clk: P5,
        pio: Pio,
        channel_a: ChA,
        channel_b: ChB,
        buffer_tx_a: &'static mut [u32; BUFFER_SIZE],
        buffer_tx_b: &'static mut [u32; BUFFER_SIZE],
        buffer_rx_a: &'static mut [u32; BUFFER_SIZE],
        buffer_rx_b: &'static mut [u32; BUFFER_SIZE],
    ) -> Self {
        // Hold down the enable pin
        pin_enable.set_low().unwrap();

        Cs4272Builder {
            pin_enable,
            pin_mclk: pin_mclk.into(),
            pin_in: pin_in.into(),
            pin_out: pin_out.into(),
            pin_ws: pin_ws.into(),
            pin_clk: pin_clk.into(),
            pio: Some(pio),
            channel_a: Some(channel_a),
            channel_b: Some(channel_b),
            buffer_tx_a: Some(buffer_tx_a),
            buffer_tx_b,
            buffer_rx_a: Some(buffer_rx_a),
            buffer_rx_b,
            system_frequency: 150.MHz(),
            sample_rate: 48.kHz(),
            interrupt: DmaInterrupt::Irq0,
        }
    }

    /// Set the driver sample rate.
    ///
    /// The CS4272 chip supports sample rates between `4kHz` and `200kHz`,
    /// defaults to `48kHz`.
    ///
    /// Note that not all sample rates are possible since the PIO division
    /// is based on the microcontroller clock speed, some deviation from the
    /// set sample rate should be expected or accounted for.
    pub fn sample_rate(mut self, sample_rate: fugit::HertzU32) -> Self {
        self.sample_rate = sample_rate;
        self
    }

    /// Set the RP235x system frequency.
    ///
    /// In case you have changed the frequency of the system you should update this value,
    /// defaults to `150MHz`.
    pub fn system_frequency(mut self, system_frequency: fugit::HertzU32) -> Self {
        self.system_frequency = system_frequency;
        self
    }

    /// Sets up interrupt `DMA_IRQ_0` for the DMA.
    pub fn set_irq0(mut self) -> Self {
        self.interrupt = DmaInterrupt::Irq0;
        self
    }

    /// Sets up interrupt `DMA_IRQ_1` for the DMA.
    pub fn set_irq1(mut self) -> Self {
        self.interrupt = DmaInterrupt::Irq1;
        self
    }
    /// Starts the PIO and DMAs, then sets the enable pin high and returns the running driver struct.
    pub fn start(
        mut self,
        resets: &mut hal::pac::RESETS,
    ) -> Cs4272<BUFFER_SIZE, Pio, ChA, ChB, P0, P1, P2, P3, P4, P5> {
        // Start the PIOs
        let (pio, sm_mclk, sm_i2s, tx, rx) = self.start_pio(resets);

        // Start the DMAs
        let (transfer_tx, transfer_rx) = self.start_dma(tx, rx);

        // Pull up the enable pin
        self.pin_enable.set_high().unwrap();

        Cs4272 {
            pio,
            sm_mclk,
            sm_i2s,
            pin_enable: self.pin_enable,
            pin_mclk: self.pin_mclk,
            pin_in: self.pin_in,
            pin_out: self.pin_out,
            pin_ws: self.pin_ws,
            pin_clk: self.pin_clk,
            buffer_tx: RefCell::new(self.buffer_tx_b),
            buffer_rx: RefCell::new(self.buffer_rx_b),
            transfer_tx: Some(transfer_tx),
            transfer_rx: Some(transfer_rx),
            buffers_ready: false,
        }
    }

    fn start_dma(
        &mut self,
        tx: Tx<(Pio, SM1)>,
        rx: Rx<(Pio, SM1)>,
    ) -> (
        Transfer<ChA, &'static mut [u32; BUFFER_SIZE], Tx<(Pio, SM1)>>,
        Transfer<ChB, Rx<(Pio, SM1)>, &'static mut [u32; BUFFER_SIZE]>,
    ) {
        let mut channel_a = self.channel_a.take().unwrap();
        let channel_b = self.channel_b.take().unwrap();
        let buffer_tx = self.buffer_tx_a.take().unwrap();
        let buffer_rx = self.buffer_rx_a.take().unwrap();

        // Enable the interrupt
        match self.interrupt {
            DmaInterrupt::Irq0 => {
                unsafe { NVIC::unmask(hal::pac::Interrupt::DMA_IRQ_0) };
                channel_a.enable_irq0();
            }
            DmaInterrupt::Irq1 => {
                unsafe { NVIC::unmask(hal::pac::Interrupt::DMA_IRQ_1) };
                channel_a.enable_irq1();
            }
        }
        // Start the DMA transfers
        let transfer_tx = single_buffer::Config::new(channel_a, buffer_tx, tx).start();
        let transfer_rx = single_buffer::Config::new(channel_b, rx, buffer_rx).start();

        (transfer_tx, transfer_rx)
    }

    fn start_pio(
        &mut self,
        resets: &mut hal::pac::RESETS,
    ) -> (
        PIO<Pio>,
        StateMachine<(Pio, SM0), Running>,
        StateMachine<(Pio, SM1), Running>,
        Tx<(Pio, SM1)>,
        Rx<(Pio, SM1)>,
    ) {
        let pio = self.pio.take().unwrap();
        let (mut pio, sm_mclk, sm_i2s, _, _) = pio.split(resets);

        let pio_mclk = pio_proc::pio_file!("src/i2s.pio", select_program("mclk"));
        let pio_i2s = pio_proc::pio_file!("src/i2s.pio", select_program("i2s_left_justified"));

        let installed_mclk = pio.install(&pio_mclk.program).unwrap();
        let installed_i2s = pio.install(&pio_i2s.program).unwrap();

        let (clk_int, clk_frac, mclk_int, mclk_frac) =
            get_pio_clock_divisions(self.system_frequency.raw(), self.sample_rate.raw());

        let (mut sm_mclk, _, _) = hal::pio::PIOBuilder::from_installed_program(installed_mclk)
            .set_pins(self.pin_mclk.id().num, 1)
            .clock_divisor_fixed_point(mclk_int, mclk_frac)
            .build(sm_mclk);

        let (mut sm_i2s, rx, tx) = hal::pio::PIOBuilder::from_installed_program(installed_i2s)
            .in_pin_base(self.pin_in.id().num)
            .out_pins(self.pin_out.id().num, 1)
            .side_set_pin_base(self.pin_ws.id().num)
            .clock_divisor_fixed_point(clk_int, clk_frac)
            .autopull(true)
            .pull_threshold(32)
            .autopush(true)
            .push_threshold(32)
            .in_shift_direction(rp235x_hal::pio::ShiftDirection::Left)
            .out_shift_direction(rp235x_hal::pio::ShiftDirection::Left)
            .build(sm_i2s);

        sm_mclk.set_pindirs([(self.pin_mclk.id().num, hal::pio::PinDir::Output)]);

        sm_i2s.set_pindirs([
            (self.pin_in.id().num, hal::pio::PinDir::Input),
            (self.pin_out.id().num, hal::pio::PinDir::Output),
            (self.pin_ws.id().num, hal::pio::PinDir::Output),
            (self.pin_clk.id().num, hal::pio::PinDir::Output),
        ]);

        // Start the PIO state machines
        let sm_mclk = sm_mclk.start();
        let sm_i2s = sm_i2s.start();

        (pio, sm_mclk, sm_i2s, tx, rx)
    }
}

/// Main driver struct, keeps ownership of all necessary resources while the driver is running
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
    /// Required method for restarting the DMA and swapping the buffers internally.
    ///
    /// ```
    /// #[interrupt]
    /// fn DMA_IRQ_0() {
    ///     cortex_m::interrupt::free(|cs| {
    ///         let mut cs4272 = CS4272.borrow(cs).borrow_mut();
    ///         cs4272.as_mut().unwrap().handle_irq();
    ///     });
    /// }
    /// ```
    pub fn handle_irq(&mut self) {
        let mut transfer_tx = self.transfer_tx.take().unwrap();

        transfer_tx.check_irq0();
        transfer_tx.check_irq1();

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

fn get_pio_clock_divisions(cpu: u32, sample_rate: u32) -> (u16, u8, u16, u8) {
    // Adjust the clock ratios according to the datasheet depending on the "speed mode"
    let (mclk_multiplier, clk_divisor) = if sample_rate < 50_000 {
        (256, 4)
    } else if sample_rate < 100_000 {
        (128, 2)
    } else {
        (64, 1)
    };

    let mclk_pio_steps_per_cycle = 2;
    let mclk_frequency = sample_rate * mclk_multiplier * mclk_pio_steps_per_cycle;
    let mclk_div = (cpu / mclk_frequency) as u16;
    let mclk_fraction = (((cpu % mclk_frequency) as f32 / mclk_frequency as f32) * 256.0) as u16;
    let clk_fraction_overflow = (mclk_fraction * clk_divisor) / 256;
    let clk_fraction = (mclk_fraction * clk_divisor) % 256;
    let clk_div = mclk_div * clk_divisor + clk_fraction_overflow;

    (clk_div, clk_fraction as u8, mclk_div, mclk_fraction as u8)
}
