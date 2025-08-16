const std = @import("std");

pub fn build(b: *std.Build) void {
    // build options
    const do_strip = b.option(
        bool,
        "strip",
        "Strip the executabes"
    ) orelse false;

    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    // encoder
    const librfxencode = b.addStaticLibrary(.{
        .name = "rfxencode",
        .target = target,
        .optimize = optimize,
        .strip = do_strip,
    });
    librfxencode.linkLibC();
    librfxencode.root_module.addCMacro("HAVE_CONFIG_H", "1");
    librfxencode.root_module.addCMacro("CONFIG_AC_H", "1");
    librfxencode.addIncludePath(b.path("."));
    librfxencode.addIncludePath(b.path("src"));
    librfxencode.addIncludePath(b.path("src/sse2"));
    librfxencode.addIncludePath(b.path("include"));
    librfxencode.addCSourceFiles(.{ .files = librfxencode_sources });
    // decoder
    const librfxdecode = b.addStaticLibrary(.{
        .name = "rfxdecode",
        .target = target,
        .optimize = optimize,
        .strip = do_strip,
    });
    librfxdecode.linkLibC();
    librfxdecode.root_module.addCMacro("HAVE_CONFIG_H", "1");
    librfxdecode.root_module.addCMacro("CONFIG_AC_H", "1");
    librfxdecode.addIncludePath(b.path("."));
    librfxdecode.addIncludePath(b.path("src"));
    librfxdecode.addIncludePath(b.path("src/sse2"));
    librfxdecode.addIncludePath(b.path("include"));
    librfxdecode.addCSourceFiles(.{ .files = librfxdecode_sources });

    b.installArtifact(librfxencode);
    b.installArtifact(librfxdecode);
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
