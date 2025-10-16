const std = @import("std");
const microzig = @import("microzig");

const MicroBuild = microzig.MicroBuild(.{
    .stm32 = true,
});

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;
    const stm32 = mb.ports.stm32;

    const fw = mb.add_firmware(.{
        .name = "VGA",
        .target = stm32.chips.STM32F103C8,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });
    //elf for debug
    mb.install_firmware(fw, .{});
}
