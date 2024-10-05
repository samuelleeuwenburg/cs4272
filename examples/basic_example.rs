#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cs4272::{Cs4272, Cs4272Builder};
use fugit::RateExtU32;
use hal::dma::{Channel, DMAExt, CH0, CH1};
use hal::gpio::bank0::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5};
use hal::gpio::{FunctionPio0, FunctionSioOutput, Pin, PullDown};
use hal::pac::{interrupt, PIO0};
use hal::singleton;
use panic_halt as _;
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const BUFFER_SIZE: usize = 32;

static CS4272: Mutex<
    RefCell<
        Option<
            Cs4272<
                BUFFER_SIZE,
                PIO0,
                Channel<CH0>,
                Channel<CH1>,
                Pin<Gpio0, FunctionSioOutput, PullDown>,
                Pin<Gpio1, FunctionPio0, PullDown>,
                Pin<Gpio2, FunctionPio0, PullDown>,
                Pin<Gpio3, FunctionPio0, PullDown>,
                Pin<Gpio4, FunctionPio0, PullDown>,
                Pin<Gpio5, FunctionPio0, PullDown>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[interrupt]
fn DMA_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        let mut cs4272 = CS4272.borrow(cs).borrow_mut();
        cs4272.as_mut().unwrap().handle_irq();
    });
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let dma = pac.DMA.split(&mut pac.RESETS);

    let cs4272k = Cs4272Builder::new(
        pins.gpio0.into_push_pull_output(),
        pins.gpio1.into_function(),
        pins.gpio2.into_function(),
        pins.gpio3.into_function(),
        pins.gpio4.into_function(),
        pins.gpio5.into_function(),
        pac.PIO0,
        dma.ch0,
        dma.ch1,
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
    )
    .start(&mut pac.RESETS);

    cortex_m::interrupt::free(|cs| {
        CS4272.borrow(cs).replace(Some(cs4272k));
    });

    loop {
        cortex_m::interrupt::free(|cs| {
            if let Some(cs4272) = CS4272.borrow(cs).borrow_mut().as_mut() {
                // Check if the buffers are ready for processing
                if cs4272.poll() {
                    // Make a copy of the input buffer for manipulation
                    let mut buffer = cs4272.get_input_buffer().clone();

                    // Process input
                    for _sample in buffer.iter_mut() {}

                    // Copy the processed buffer to the output
                    cs4272.set_output_buffer(&buffer);
                }
            }
        });
    }
}
