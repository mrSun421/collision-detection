const std = @import("std");
const rng = std.Random.DefaultPrng;
const rl = @import("raylib");

const Circle = struct { id: u64, position: rl.Vector2, velocity: rl.Vector2, radius: f32, color: rl.Color, mass: f32 };

pub fn main() !void {
    const screenWidth = 1600;
    const screenHeight = 900;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};

    var circleArray = std.MultiArrayList(Circle){};
    defer circleArray.deinit(gpa.allocator());
    var rnd = rng.init(0);

    while (circleArray.len < 1000) {
        const radius = 2.0 + rnd.random().float(f32) * 4.0;
        const xpos = radius + rnd.random().float(f32) * (screenWidth - radius);
        const ypos = radius + rnd.random().float(f32) * (screenHeight - radius);
        const pos: rl.Vector2 = rl.Vector2.init(xpos, ypos);
        const vel = 50.0 + rnd.random().float(f32) * 5.0;
        const randAng = rnd.random().float(f32) * 2 * std.math.pi;
        const mass = 10.0 + rnd.random().float(f32) * 100.0;
        const hue = 130.0 + rnd.random().float(f32) * 100.0;
        const saturation = 0.5 + rnd.random().float(f32) * 0.5;
        const brightness = 0.5 + rnd.random().float(f32) * 0.5;
        const col = rl.Color.fromHSV(hue, saturation, brightness);
        const newCircle: Circle = .{ .id = circleArray.len, .position = pos, .velocity = rl.Vector2.init(@cos(randAng), @sin(randAng)).scale(vel), .radius = radius, .color = col, .mass = mass };
        try circleArray.append(gpa.allocator(), newCircle);
    }

    rl.initWindow(screenWidth, screenHeight, "collision detection Sim");
    defer rl.closeWindow();

    rl.setTargetFPS(120);

    while (!rl.windowShouldClose()) {
        {
            const dt = rl.getFrameTime();
            for (circleArray.items(.position), circleArray.items(.velocity), circleArray.items(.radius)) |*pos, *vel, rad| {
                const newPos: rl.Vector2 = pos.add(vel.scale(dt));
                const left = newPos.x - rad;
                const right = newPos.x + rad;
                const top = newPos.y - rad;
                const bottom = newPos.y + rad;
                if ((left <= 0) or (right >= screenWidth)) {
                    vel.*.x = -vel.x;
                }
                if ((bottom >= screenHeight) or (top <= 0)) {
                    vel.*.y = -vel.y;
                }
                pos.* = newPos;
            }

            var spacePartition = std.AutoHashMap(struct { i64, i64 }, std.ArrayList(u64)).init(gpa.allocator());
            defer {
                var iter = spacePartition.iterator();
                while (iter.next()) |entry| {
                    entry.value_ptr.deinit();
                }

                spacePartition.deinit();
            }

            for (0..circleArray.len) |i| {
                const circ = circleArray.get(i);
                const offsets = [_]rl.Vector2{ rl.Vector2.init(circ.radius, circ.radius), rl.Vector2.init(-circ.radius, circ.radius), rl.Vector2.init(circ.radius, -circ.radius), rl.Vector2.init(-circ.radius, -circ.radius) };
                for (offsets) |offset| {
                    const corner = circ.position.add(offset);
                    const spacePartitionRegion: struct { i32, i32 } = .{ @intFromFloat(@floor(corner.x / 16)), @intFromFloat(@floor(corner.y / 16)) };
                    var setOfCircles = try spacePartition.getOrPut(spacePartitionRegion);
                    if (setOfCircles.found_existing) {
                        try setOfCircles.value_ptr.append(i);
                    } else {
                        setOfCircles.value_ptr.* = std.ArrayList(u64).init(gpa.allocator());
                        try setOfCircles.value_ptr.append(i);
                    }
                }
            }

            var spacePartitionIter = spacePartition.iterator();
            while (spacePartitionIter.next()) |entry| {
                const setOfCircles = entry.value_ptr;
                for (setOfCircles.items) |i| {
                    for (setOfCircles.items) |j| {
                        var circle1 = circleArray.get(i);
                        var circle2 = circleArray.get(j);
                        if (circle1.id == circle2.id) {
                            continue;
                        } else {
                            const dist = circle1.position.distance(circle2.position);
                            if (dist <= circle1.radius + circle2.radius) {
                                const pos1 = circle1.position;
                                const pos2 = circle2.position;
                                const vel1 = circle1.velocity;
                                const vel2 = circle2.velocity;
                                const mass1 = circle1.mass;
                                const mass2 = circle2.mass;
                                const x1subx2: rl.Vector2 = pos1.subtract(pos2);
                                const v1subv2: rl.Vector2 = vel1.subtract(vel2);
                                const x2subx1: rl.Vector2 = pos2.subtract(pos1);
                                const v2subv1: rl.Vector2 = vel2.subtract(vel1);
                                const newVel1 = vel1.subtract(x1subx2.scale((2.0 * mass2 / (mass1 + mass2)) * (v1subv2.dotProduct(x1subx2) / x1subx2.lengthSqr())));
                                const newVel2 = vel2.subtract(x2subx1.scale((2.0 * mass1 / (mass1 + mass2)) * (v2subv1.dotProduct(x2subx1) / x2subx1.lengthSqr())));
                                circle1.velocity = newVel1;
                                circle2.velocity = newVel2;
                                const posOffset = dist - (circle1.radius + circle2.radius);
                                const newPos1 = pos1.add(x2subx1.normalize().scale(posOffset / 2.0));
                                const newPos2 = pos2.add(x1subx2.normalize().scale(posOffset / 2.0));
                                circle1.position = newPos1;
                                circle2.position = newPos2;
                                circleArray.set(i, circle1);
                                circleArray.set(j, circle2);
                            }
                        }
                    }
                }
            }
        }
        {
            rl.beginDrawing();
            defer rl.endDrawing();
            rl.clearBackground(rl.Color.ray_white);
            for (circleArray.items(.position), circleArray.items(.radius), circleArray.items(.color)) |pos, rad, col| {
                rl.drawCircleV(pos, rad, col);
            }
            var buf: [64]u8 = undefined;
            const formattedFrametime = try std.fmt.bufPrintZ(&buf, "{d} ms", .{rl.getFrameTime() * 1000});
            rl.drawText(formattedFrametime, 0, 0, 32, rl.Color.dark_gray);
        }
    }
}
