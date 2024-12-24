const std = @import("std");
const rng = std.Random.DefaultPrng;
const rl = @import("raylib");

const circleCount = 1000;
const Circle = struct { id: usize, position: rl.Vector2, velocity: rl.Vector2, radius: f32, color: rl.Color, mass: f32 };
const KDTreeNode = struct { left: ?*const KDTreeNode, right: ?*const KDTreeNode, circleIndices: ?[]usize };
const IndexPosition = std.meta.Tuple(&.{ usize, rl.Vector2 });

pub fn main() !void {
    const screenWidth = 1600;
    const screenHeight = 900;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};

    var circleArray = std.MultiArrayList(Circle){};
    try circleArray.setCapacity(gpa.allocator(), circleCount);
    defer circleArray.deinit(gpa.allocator());
    var rnd = rng.init(0);

    while (circleArray.len < circleCount) {
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
            // try spacePartitionMethod(gpa.allocator(), &circleArray);
            try kdTreeMethod(gpa.allocator(), &circleArray);
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

fn spacePartitionMethod(allocator: std.mem.Allocator, circleArray: *std.MultiArrayList(Circle)) !void {
    var arena = std.heap.ArenaAllocator.init(allocator);
    defer arena.deinit();

    var spacePartition = std.AutoHashMap(struct { i64, i64 }, std.ArrayList(usize)).init(arena.allocator());

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
                setOfCircles.value_ptr.* = std.ArrayList(usize).init(arena.allocator());
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

fn kdTreeMethod(allocator: std.mem.Allocator, circleArray: *std.MultiArrayList(Circle)) !void {
    var arena = std.heap.ArenaAllocator.init(allocator);
    defer arena.deinit();
    const head = try createKDTree(arena.allocator(), circleArray, 3);
    try traverseKDandUpdate(&head, circleArray);
}

fn traverseKDandUpdate(head: ?*const KDTreeNode, circleArray: *std.MultiArrayList(Circle)) !void {
    if (head) |headVal| {
        if (headVal.circleIndices) |circIdxes| {
            for (circIdxes) |i| {
                for (circIdxes) |j| {
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

        try traverseKDandUpdate(headVal.left, circleArray);
        try traverseKDandUpdate(headVal.right, circleArray);
    }
}

fn createKDTree(allocator: std.mem.Allocator, circleArray: *std.MultiArrayList(Circle), maxDepth: usize) !KDTreeNode {
    var indexAndPositions: std.ArrayList(IndexPosition) = try std.ArrayList(IndexPosition).initCapacity(allocator, circleArray.len);
    for (circleArray.items(.position), 0..) |pos, i| {
        try indexAndPositions.append(.{ i, pos });
    }
    return try innerCreateKDTree(allocator, try indexAndPositions.toOwnedSlice(), 0, maxDepth);
}

fn innerCreateKDTree(allocator: std.mem.Allocator, indexAndPositions: []IndexPosition, depth: usize, maxDepth: usize) !KDTreeNode {
    if (depth >= maxDepth) {
        var indicies: std.ArrayList(usize) = try std.ArrayList(usize).initCapacity(allocator, indexAndPositions.len);
        for (indexAndPositions) |val| {
            const valIdx: usize = val[0];
            try indicies.append(valIdx);
        }
        const node = KDTreeNode{ .left = null, .right = null, .circleIndices = try indicies.toOwnedSlice() };
        return node;
    }

    const first = indexAndPositions[0];
    const firstPos: rl.Vector2 = first[1];
    const rest = indexAndPositions[1..];
    var leftIndexAndPositions = std.ArrayList(IndexPosition).init(allocator);
    var rightIndexAndPositions = std.ArrayList(IndexPosition).init(allocator);
    for (rest) |val| {
        const valPos: rl.Vector2 = val[1];
        if (depth % 2 == 0) {
            if (valPos.x < firstPos.x) {
                try leftIndexAndPositions.append(val);
            } else {
                try rightIndexAndPositions.append(val);
            }
        } else {
            if (valPos.y < firstPos.y) {
                try leftIndexAndPositions.append(val);
            } else {
                try rightIndexAndPositions.append(val);
            }
        }
        try leftIndexAndPositions.append(first);
        try rightIndexAndPositions.append(first);
    }
    const leftNode = try innerCreateKDTree(allocator, try leftIndexAndPositions.toOwnedSlice(), depth + 1, maxDepth);
    const rightNode = try innerCreateKDTree(allocator, try rightIndexAndPositions.toOwnedSlice(), depth + 1, maxDepth);
    const head = KDTreeNode{ .left = &leftNode, .right = &rightNode, .circleIndices = null };
    return head;
}
