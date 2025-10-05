const std = @import("std");
const builtin = @import("builtin");

pub fn build(b: *std.Build) void {
    // build options
    const do_strip = b.option(
        bool,
        "strip",
        "Strip the executabes"
    ) orelse false;

    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const libflags = if (optimize == .Debug) "-DDEBUG=1" else "-DNDEBUG=1";
    // encoder
    const librfxencode = myAddStaticLibrary(b, "rfxencode", target,
            optimize, do_strip);
    librfxencode.linkLibC();
    librfxencode.addIncludePath(b.path("."));
    librfxencode.addIncludePath(b.path("src"));
    librfxencode.addIncludePath(b.path("include"));
    librfxencode.addCSourceFiles(.{ .files = librfxencode_sources,
            .flags =  &.{libflags} });
    // decoder
    const librfxdecode = myAddStaticLibrary(b, "rfxdecode", target,
            optimize, do_strip);
    librfxdecode.linkLibC();
    librfxdecode.addIncludePath(b.path("."));
    librfxdecode.addIncludePath(b.path("src"));
    librfxdecode.addIncludePath(b.path("include"));
    librfxdecode.addCSourceFiles(.{ .files = librfxdecode_sources,
            .flags =  &.{libflags} });

    b.installArtifact(librfxencode);
    b.installArtifact(librfxdecode);
}

//*****************************************************************************
fn myAddStaticLibrary(b: *std.Build, name: []const u8,
        target: std.Build.ResolvedTarget,
        optimize: std.builtin.OptimizeMode,
        do_strip: bool) *std.Build.Step.Compile
{
    if ((builtin.zig_version.major == 0) and (builtin.zig_version.minor < 15))
    {
        return b.addStaticLibrary(.{
            .name = name,
            .target = target,
            .optimize = optimize,
            .strip = do_strip,
        });
    }
    return b.addLibrary(.{
        .name = name,
        .root_module = b.addModule(name, .{
            .target = target,
            .optimize = optimize,
            .strip = do_strip,
        }),
        .linkage = .static,
    });
}

const librfxencode_sources = &.{
    "src/rfxencode.c",
    "src/rfxencode_compose.c",
    "src/rfxencode_tile.c",
    "src/rfxencode_dwt.c",
    "src/rfxencode_quantization.c",
    "src/rfxencode_differential.c",
    "src/rfxencode_rlgr1.c",
    "src/rfxencode_rlgr3.c",
    "src/rfxencode_alpha.c",
    "src/rfxencode_diff_rlgr1.c",
    "src/rfxencode_diff_rlgr3.c",
    "src/rfxencode_rgb_to_yuv.c",
    "src/rfxencode_dwt_rem.c",
    "src/rfxencode_dwt_shift_rem.c",
};

const librfxdecode_sources = &.{
    "src/rfxdecode.c",
    "src/rfxdecode_decompose.c",
    "src/rfxdecode_tile.c",
    "src/rfxdecode_dwt.c",
    "src/rfxdecode_quantization.c",
    "src/rfxdecode_differential.c",
    "src/rfxdecode_rlgr1.c",
    "src/rfxdecode_rlgr3.c",
    "src/rfxdecode_alpha.c",
    "src/rfxdecode_rlgr1_diff.c",
    "src/rfxdecode_rlgr3_diff.c",
};
