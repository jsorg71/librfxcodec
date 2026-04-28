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
    myLinkLibC(librfxencode);
    myAddIncludePath(librfxencode, b.path("."));
    myAddIncludePath(librfxencode, b.path("src"));
    myAddIncludePath(librfxencode, b.path("include"));
    myAddCSourceFiles(librfxencode, .{ .files = librfxencode_sources, .flags =  &.{libflags} });
    // decoder
    const librfxdecode = myAddStaticLibrary(b, "rfxdecode", target,
            optimize, do_strip);
    myLinkLibC(librfxdecode);
    myAddIncludePath(librfxdecode, b.path("."));
    myAddIncludePath(librfxdecode, b.path("src"));
    myAddIncludePath(librfxdecode, b.path("include"));
    myAddCSourceFiles(librfxdecode, .{ .files = librfxdecode_sources, .flags =  &.{libflags} });

    b.installArtifact(librfxencode);
    b.installArtifact(librfxdecode);
}

//*****************************************************************************
fn myLinkLibC(compile: *std.Build.Step.Compile) void
{
    if ((builtin.zig_version.major == 0) and (builtin.zig_version.minor < 16))
    {
        compile.linkLibC();
    }
    else
    {
        compile.root_module.link_libc = true;
    }
}

//*****************************************************************************
fn myAddIncludePath(compile: *std.Build.Step.Compile, lazy_path: std.Build.LazyPath) void
{
    if ((builtin.zig_version.major == 0) and (builtin.zig_version.minor < 16))
    {
        compile.addIncludePath(lazy_path);
    }
    else
    {
        compile.root_module.addIncludePath(lazy_path);
    }
}

//*****************************************************************************
fn myAddCSourceFiles(compile: *std.Build.Step.Compile, options: std.Build.Module.AddCSourceFilesOptions) void
{
    if ((builtin.zig_version.major == 0) and (builtin.zig_version.minor < 16))
    {
        compile.addCSourceFiles(options);
    }
    else
    {
        compile.root_module.addCSourceFiles(options);
    }
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
