# CS4272 Crate

I2S implementation for the
[Cirrus Logic CS4272](https://statics.cirrus.com/pubs/proDatasheet/CS4272_F2.pdf) IC
built using the PIO.

The driver assumes the CS4272 is running in left-justified and slave mode.

This driver uses double buffering for both input and output: when one set of buffers is busy
reading/writing to and from the hardware the other set is available to be read and written to.

The library requires you to handle the switching of the buffers using the `DMA_IRQ_0`
so it makes sense to put the driver into a `Mutex<RefCell<Option<Cs4272>>` for concurrent access.

```rust
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

                Pin<Gpio5, FunctionPio0, PullDown>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));
```

And handle the interrupt:

```rust
#[interrupt]
fn DMA_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(driver) = CS4272.borrow(cs).borrow_mut().as_mut() {
            driver.handle_irq0();
        }
    });
}
```

Configure and start the driver:

```rust
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
```

Using the [Cs4272::poll] method you can check wether or not the current available buffer has already
been written to. This makes it relatively easy to process the data whenever it makes sense for your
application:

```rust
cortex_m::interrupt::free(|cs| {
    if let Some(cs4272) = CS4272.borrow(cs).borrow_mut().as_mut() {
        // Check if the buffers are ready for processing
        if cs4272.poll() {
            // Make a copy of the input buffer for manipulation
            let mut buffer = cs4272.get_input_buffer().clone();

            // Process input
            for _sample in buffer.iter_mut() { /* ... */ }

            // Copy the processed buffer to the output
            cs4272.set_output_buffer(&input);
        }
    }
});
```
Just make sure to keep the processing light or clone the data out of the `interrupt::free`
callback so you won't miss a buffer.
