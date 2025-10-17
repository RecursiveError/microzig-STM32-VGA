const point_to_check = [8][2]i8{
    // UP row
    .{ -1, -1 },
    .{ -1, 0 },
    .{ -1, 1 },

    //mid row
    .{ 0, -1 },
    .{ 0, 1 },

    //DOWN ROW
    .{ 1, -1 },
    .{ 1, 0 },
    .{ 1, 1 },
};

pub fn proscess_life(buf: []const u8, out: []u8, v: usize, h: usize) void {
    const max_y = @as(i32, @intCast(v));
    const max_x = @as(i32, @intCast(h));
    for (0..v) |lidx| {
        for (0..h) |bidx| {
            const line = (lidx * h);
            const byte_idx = line + bidx;
            var n_count: usize = 0;
            for (point_to_check) |point| {
                const y = @as(i32, @intCast(lidx)) + point[0];
                const x = @as(i32, @intCast(bidx)) + point[1];
                if ((y < 0) or (x < 0)) continue;
                if ((y >= max_y) or (x >= max_x)) continue;
                const n_idx = @as(usize, @intCast(y)) * h + @as(usize, @intCast(x));
                const n_val = buf[n_idx];
                if (n_val != 0) n_count += 1;
            }

            if (buf[byte_idx] != 0) {
                if (n_count < 2 or n_count > 3) out[byte_idx] = 0 else out[byte_idx] = 0xFF;
            } else {
                if (n_count == 3) out[byte_idx] = 0xFF;
            }
        }
    }
}
