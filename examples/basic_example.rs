#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cs4272::Cs4272;
use hal::clocks::{Clock, ClocksManager};
use hal::dma::{Channel, DMAExt, CH0, CH1};
use hal::gpio::{bank0, FunctionPio0, FunctionSioOutput, Pin, PullDown};
use hal::pac::interrupt;
use hal::singleton;
use panic_halt as _;
use rp235x_hal as hal;
use rp235x_hal::fugit::RateExtU32;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const BUFFER_SIZE: usize = 512;

static CS4272: Mutex<
    RefCell<
        Option<
            Cs4272<
                BUFFER_SIZE,
                hal::pac::PIO0,
                Channel<CH0>,
                Channel<CH1>,
                Pin<bank0::Gpio0, FunctionSioOutput, PullDown>,
                Pin<bank0::Gpio1, FunctionPio0, PullDown>,
                Pin<bank0::Gpio2, FunctionPio0, PullDown>,
                Pin<bank0::Gpio3, FunctionPio0, PullDown>,
                Pin<bank0::Gpio4, FunctionPio0, PullDown>,
                Pin<bank0::Gpio5, FunctionPio0, PullDown>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let sio = hal::Sio::new(pac.SIO);
    let clocks = ClocksManager::new(pac.CLOCKS);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let dma = pac.DMA.split(&mut pac.RESETS);

    let cs4272k = Cs4272::new(
        pac.PIO0,
        dma.ch0,
        dma.ch1,
        pins.gpio0.into_push_pull_output(),
        pins.gpio1.into_function(),
        pins.gpio2.into_function(),
        pins.gpio3.into_function(),
        pins.gpio4.into_function(),
        pins.gpio5.into_function(),
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
        singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
        48.kHz(),
    );

    cortex_m::interrupt::free(|cs| {
        CS4272.borrow(cs).replace(Some(cs4272k));
    });

    loop {
        let buffers_ready: bool =
            cortex_m::interrupt::free(|cs| CS4272.borrow(cs).borrow_mut().as_ref().unwrap().poll());

        // Check if the buffers are ready for processing
        if buffers_ready {
            // Make a copy of the input buffer for manipulation
            let mut input = cortex_m::interrupt::free(|cs| {
                CS4272
                    .borrow(cs)
                    .borrow_mut()
                    .as_ref()
                    .unwrap()
                    .get_input_buffer(cs)
                    .clone()
            });

            // Scale down the input
            for sample in input.iter_mut() {
                *sample = *sample / 2;
            }

            cortex_m::interrupt::free(|cs| {
                if let Some(cs4272) = CS4272.borrow(cs).borrow_mut().as_mut() {
                    cs4272.set_output_buffer(cs, &input);
                }
            });
        }
    }
}

#[interrupt]
fn DMA_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(driver) = CS4272.borrow(cs).borrow_mut().as_mut() {
            driver.handle_irq0(cs);
        }
    });
}

#[interrupt]
fn DMA_IRQ_1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(driver) = CS4272.borrow(cs).borrow_mut().as_mut() {
            driver.handle_irq1(cs);
        }
    });
}
