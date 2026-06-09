//! Build the vendored GTSAM (third_party/gtsam, a pinned git submodule) and the cxx shim.
//!
//! GTSAM is configured CPU-only, Boost-free, and **static**, so the resulting Rust
//! binaries have no runtime C++ dependency and the build works on any machine with a
//! C++17 toolchain + CMake — no system GTSAM, no ROS (ADR 0001/0003).
//!
//! Escape hatch: set `SLAM_GTSAM_PREFIX` to an existing GTSAM install prefix (containing
//! `include/` and `lib/`) to skip the vendored build entirely.

use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    println!("cargo:rerun-if-env-changed=SLAM_GTSAM_PREFIX");
    println!("cargo:rerun-if-changed=cpp/shim.h");
    println!("cargo:rerun-if-changed=cpp/shim.cpp");
    println!("cargo:rerun-if-changed=src/lib.rs");

    let prefix = match env::var_os("SLAM_GTSAM_PREFIX") {
        Some(p) => PathBuf::from(p),
        None => build_vendored_gtsam(&manifest_dir),
    };

    let include = prefix.join("include");
    // GTSAM installs its bundled Eigen under include/gtsam/3rdparty/Eigen.
    let bundled_eigen = include.join("gtsam/3rdparty/Eigen");

    let mut shim = cxx_build::bridge("src/lib.rs");
    shim.file("cpp/shim.cpp")
        .include(&include)
        .std("c++17")
        // GTSAM/Eigen headers are third-party: their warnings are not ours to fix.
        .flag_if_supported("-Wno-deprecated-copy")
        .flag_if_supported("-Wno-unused-parameter");
    if bundled_eigen.is_dir() {
        shim.include(&bundled_eigen);
    }
    shim.compile("slam-gtsam-shim");

    // Link every static archive GTSAM installed (gtsam first: it depends on the bundled
    // metis/cephes archives, and the linker resolves left to right).
    let libdir = ["lib", "lib64"]
        .iter()
        .map(|d| prefix.join(d))
        .find(|p| p.is_dir())
        .unwrap_or_else(|| panic!("no lib/ under GTSAM prefix {}", prefix.display()));
    println!("cargo:rustc-link-search=native={}", libdir.display());
    let mut archives: Vec<String> = std::fs::read_dir(&libdir)
        .unwrap()
        .filter_map(|e| {
            let name = e.unwrap().file_name().into_string().ok()?;
            let stem = name.strip_prefix("lib")?.strip_suffix(".a")?;
            Some(stem.to_string())
        })
        .collect();
    archives.sort_by_key(|a| (a != "gtsam", a.clone()));
    if !archives.iter().any(|a| a == "gtsam") {
        panic!("libgtsam.a not found in {}", libdir.display());
    }
    for archive in archives {
        println!("cargo:rustc-link-lib=static={archive}");
    }
}

fn build_vendored_gtsam(manifest_dir: &std::path::Path) -> PathBuf {
    let gtsam_src = manifest_dir.join("../../third_party/gtsam");
    assert!(
        gtsam_src.join("CMakeLists.txt").exists(),
        "GTSAM submodule not found at {}; run `git submodule update --init`",
        gtsam_src.display()
    );

    cmake::Config::new(&gtsam_src)
        // Always optimised, even for `cargo build` (debug GTSAM is unusably slow and the
        // shim is the debugging surface, not GTSAM internals).
        .profile("Release")
        .define("BUILD_SHARED_LIBS", "OFF")
        // Boost-free build (GTSAM >= 4.3).
        .define("GTSAM_ENABLE_BOOST_SERIALIZATION", "OFF")
        .define("GTSAM_USE_BOOST_FEATURES", "OFF")
        // CPU-only, library-only.
        .define("GTSAM_WITH_TBB", "OFF")
        .define("GTSAM_WITH_EIGEN_MKL", "OFF")
        .define("GTSAM_BUILD_TESTS", "OFF")
        .define("GTSAM_BUILD_EXAMPLES_ALWAYS", "OFF")
        .define("GTSAM_BUILD_TIMING_ALWAYS", "OFF")
        .define("GTSAM_BUILD_UNSTABLE", "OFF")
        .define("GTSAM_BUILD_PYTHON", "OFF")
        .define("GTSAM_FORCE_STATIC_LIB", "ON")
        .build()
}
