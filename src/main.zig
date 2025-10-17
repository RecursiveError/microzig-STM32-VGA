const microzig = @import("microzig");
const std = @import("std");
const GoL = @import("life.zig");
const hal = microzig.hal;
const rcc = hal.rcc;
const gpio = hal.gpio;
const timer = hal.timer;
const time = hal.time;
const dma = hal.dma;

// The STM32F103 not only lacks enough memory to store a full VGA display buffer,
// but also does not have peripherals to keep up with the pixel clock.
// So the technique is to repeat each pixel N times, effectively lowering the resolution.
// In this example we are in SVGA 800x600 @56Hz mode (pixel clock 36MHz).
// By repeating each pixel 8 times, we reduce it to 100x75 @56Hz (pixel clock 4.5MHz).
var PIXEL_BUFFER: [75][100]u8 = undefined;
var PIXEL_BUFFER2: [75 * 100]u8 = undefined;

const PIXEL_PER_LINE = 1024 / 8; // front porch + 800 +back porch + hsync
const PIXEL_DIV = 8;

//======= Peripheral Configuration =========
const PIXEL_OUT_BUS = &microzig.chip.peripherals.GPIOA.ODR;

// Timer responsible for synchronizing all peripherals to the desired pixel clock
const PIXEL_CLK = timer.GPTimer.init(.TIM2);
const PIXEL_CLOCK_CONF = timer.TimerGenealConfig{
    .enable_update_dma_request = true,
    .counter_mode = .down,
    .trigger_output = .Update,

    //72Mhz/2 = 36Mhz | 36Mhz/8 = tick freq 4.5Mhz
    .auto_reload = 8,
    .prescaler = 1,
};

// Clock responsible for line management.
// ITR1 synchronizes TIM3 with TIM2, our PIXEL Clock
const HSYNC_CLK = timer.GPTimer.init(.TIM3);
const HSYNC_CLOCK_CONF = timer.TimerGenealConfig{
    .counter_mode = .up,
    .sync_config = .{
        .trigger_source = .ITR1,
        .mode = .ExternalClockMode1,
    },
    .auto_reload = PIXEL_PER_LINE - 1,
};

// Configures channel 3 for the end of BackPorch event
const BACK_PORCH_CH = 2;
const BACK_PORCH_END = 128 / PIXEL_DIV;
const BACK_PORCH_CONF = timer.CCConfig{
    .ch_mode = .{
        .compare = .{
            .mode = .ActiveOnMatch,
        },
    },
    .channel_interrupt_enable = true,
};

// configures channel 2 for the front porch start event
const FRONT_PORCH_CH = 0;
const FRONT_PORCH_START = BACK_PORCH_END + 100;
const FRONT_PORCH_CONF = timer.CCConfig{
    .ch_mode = .{
        .compare = .{
            .mode = .ActiveOnMatch,
        },
    },
    .channel_interrupt_enable = true,
};

// Configures channel 2 for Hsync pulse
const HSYNC_PULSE_CH = 1;
const HSYNC_PULSE_LEN = PIXEL_PER_LINE - (72 / PIXEL_DIV);
const HSYNC_PULSE_CONF = timer.CCConfig{
    .ch_mode = .{
        .compare = .{
            .mode = .PwmMode1,
            .pre_load = true,
            .fast_mode = true,
        },
    },
};

const PX_DMA_CH = dma.DMAChannel.init(.DMA1, 1);
const PX_DMA_CONF = dma.Config{
    .direction = .FromMemory,
    .memory_increment = true,
    .peripheral_size = .Bits8,
    .memory_size = .Bits8,
    .periph_address = @intFromPtr(PIXEL_OUT_BUS),
    .mem_address = 0,
    .transfer_count = 0,
    .priority = .VeryHigh,
};

// configures the system clock to 72MHz
const SYSCLK = rcc.Config{
    .HSEOSC = @enumFromInt(8_000_000),
    .PLLSource = .RCC_PLLSOURCE_HSE,
    .PLLMUL = .RCC_PLL_MUL9,
    .SysClkSource = .RCC_SYSCLKSOURCE_PLLCLK,
    .APB1Prescaler = .RCC_HCLK_DIV2,
};

//  ======= GPIOs =======
const R0 = gpio.Pin.from_port(.A, 0);
const B0 = gpio.Pin.from_port(.A, 1);
const G0 = gpio.Pin.from_port(.A, 2);
const hsync_pin = gpio.Pin.from_port(.B, 5);
const vsync_pin = gpio.Pin.from_port(.B, 12);

// ISR handler
var real_lines: usize = 0; //counts the actual lines to generate Vsync
var line: usize = 0; //current buffer line
var repeat: usize = PIXEL_DIV; //since we divided the resolution by 8, we must repeat the line 8x to keep the proportion
var delta: u64 = 0; //frame count
pub fn vga_handler() callconv(.c) void {
    const event = HSYNC_CLK.get_interrupt_flags();

    if (event.channel1) {

        // Front Porch event
        // forces DMA to stop, turns off outputs on GPIOs and sets up the next line
        real_lines += 1;
        PX_DMA_CH.stop();
        PIXEL_OUT_BUS.write_raw(0);
        if (repeat == 0) {
            line += 1;
            line %= PIXEL_BUFFER.len;
            repeat = PIXEL_DIV;
        } else {
            repeat -= 1;
        }

        PX_DMA_CH.set_memory_address(@intFromPtr(&PIXEL_BUFFER[line]));
        PX_DMA_CH.set_count(100);
    } else if (event.channel3) {
        // Back Porch event
        // based on the number of real lines, checks which part of Vsync we are in

        // if in visible lines, start DMA
        if (real_lines < 600) {
            PX_DMA_CH.start();
        } else if (real_lines == 600) { // Start of Vsync Front Porch
            PX_DMA_CH.stop();
            PIXEL_OUT_BUS.write_raw(0x00);
        } else if (real_lines == 601) { // Start of Vsync Pulse
            vsync_pin.put(1);
        } else if (real_lines == 603) { // End of Vsync pulse
            vsync_pin.put(0);
            line = 0;
            repeat = PIXEL_DIV - 1;
            delta += 1;
        } else if (real_lines == 625) { // End of Vsync back porch
            real_lines = 0;
        }
    }
    HSYNC_CLK.clear_interrupts();
}

pub const microzig_options = microzig.Options{
    .logFn = hal.uart.log,
    .interrupts = .{
        .TIM3 = .{ .c = vga_handler },
    },
};

pub fn main() !void {

    //clk init
    try rcc.apply_clock(SYSCLK);
    rcc.enable_clock(.TIM2);
    rcc.enable_clock(.TIM3);
    rcc.enable_clock(.TIM4);

    rcc.enable_clock(.GPIOA);
    rcc.enable_clock(.GPIOB);
    rcc.enable_clock(.DMA1);
    rcc.enable_clock(.AFIO);

    //timer 4 for general time counting, not part of VGA
    hal.time.init_timer(.TIM4);
    // Moves the Hsync channel pin to B5 to completely free GPIOA for VGA
    gpio.apply_remap(.{ .TIM3 = .Partial_Remap_2 });

    for (&[_]gpio.Pin{ R0, B0, G0, vsync_pin }) |pin| {
        pin.set_output_mode(.general_purpose_push_pull, .max_50MHz);
    }
    hsync_pin.set_output_mode(.alternate_function_push_pull, .max_50MHz);

    PIXEL_CLK.timer_general_config(PIXEL_CLOCK_CONF);
    HSYNC_CLK.timer_general_config(HSYNC_CLOCK_CONF);

    HSYNC_CLK.configure_ccr(FRONT_PORCH_CH, FRONT_PORCH_CONF);
    HSYNC_CLK.load_ccr(FRONT_PORCH_CH, FRONT_PORCH_START);
    HSYNC_CLK.set_channel_interrupt(FRONT_PORCH_CH, true);

    HSYNC_CLK.configure_ccr(BACK_PORCH_CH, BACK_PORCH_CONF);
    HSYNC_CLK.load_ccr(BACK_PORCH_CH, BACK_PORCH_END);
    HSYNC_CLK.set_channel_interrupt(BACK_PORCH_CH, true);

    HSYNC_CLK.configure_ccr(HSYNC_PULSE_CH, HSYNC_PULSE_CONF);
    HSYNC_CLK.set_channel(HSYNC_PULSE_CH, true);
    HSYNC_CLK.load_ccr(HSYNC_PULSE_CH, HSYNC_PULSE_LEN);

    HSYNC_CLK.software_update();

    PX_DMA_CH.apply(PX_DMA_CONF);
    microzig.interrupt.enable_interrupts();
    microzig.interrupt.enable(.TIM3);

    HSYNC_CLK.start();
    PIXEL_CLK.start();

    PIXEL_BUFFER[30][50] = 0xFF;
    PIXEL_BUFFER[30][52] = 0xFF;
    PIXEL_BUFFER[31][50] = 0xFF;
    PIXEL_BUFFER[31][51] = 0xFF;
    PIXEL_BUFFER[31][52] = 0xFF;
    PIXEL_BUFFER[32][51] = 0xFF;

    PIXEL_BUFFER[40][10] = 0xFF;
    PIXEL_BUFFER[40][12] = 0xFF;
    PIXEL_BUFFER[41][10] = 0xFF;
    PIXEL_BUFFER[41][11] = 0xFF;
    PIXEL_BUFFER[41][12] = 0xFF;
    PIXEL_BUFFER[42][11] = 0xFF;

    time.sleep_ms(6500);
    const delta_ref: *volatile u64 = &delta;
    while (true) {
        GoL.proscess_life(@ptrCast(&PIXEL_BUFFER), &PIXEL_BUFFER2, PIXEL_BUFFER.len, PIXEL_BUFFER[0].len);
        PIXEL_BUFFER2[@as(usize, @intCast(delta_ref.* % PIXEL_BUFFER2.len))] = 0xFF;
        @memcpy(@as([]u8, @ptrCast(&PIXEL_BUFFER)), &PIXEL_BUFFER2);
    }
}
