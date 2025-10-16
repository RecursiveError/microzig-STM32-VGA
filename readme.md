# VGA STM32F103 
This is a small example of how to generate a VGA signal on the STM32F103.

Before anything else, you need to understand the VGA protocol. This video [explains it well](https://youtu.be/BUTHtNrpwiI?si=hduxxu8qLjxT3YDS).

## How it works
This example uses 2 general-purpose timers and one DMA channel.

One timer acts as the master for the other timer and the DMA, synchronizing everything at the desired pixel clock.
The slave timer uses 3 channels to manage Back and Front Porch events and also generates a PWM signal for Hsync and a software PWM for Vsync.

The DMA, also synchronized with the pixel clock, sends the image buffer data directly to GPIOA.ODR, so you can effectively have up to 16 bits of data, but this example only uses 3 bits.
(You may need to adjust your VGA monitor settings since the timing is not perfect.)

### Pin out

RED0 = A0
BLUE0 = A1
GREN0 = A2
hsync_pin = B5
vsync_pin = B12

## NOTE
This type of task is not practical at all for old and limited controllers like the F103. Issues such as distortion are common, and although it consumes little CPU time, having other tasks like memory access or interrupts will intensify image distortion, because you will be competing for memory access with the DMA and spending extra cycles handling timer interrupts.

You can try to reduce this by using techniques like ping-pong buffers to avoid directly competing with the DMA when drawing to the image buffer, or by having a dedicated timer just for Vsync and Hsync to generate shorter, faster interrupts for display events, but the improvement is minimal. Overall, this is an educational project and not something that could be used in the real world, so the code quality is "good enough."

### Note 2:
At the time I am writing this example, the DMA in the STM32F103 HAL for MicroZig is broken... or rather, I broke the DMA since I was the one who modified it :P, but it will probably be fixed soon, so pay attention to your MicroZig version.