use std::{
    collections::hash_map::DefaultHasher,
    env,
    hash::{Hash, Hasher},
    path::PathBuf,
    process::Command,
};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=native");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=HORUS_NATIVE_ROS_LIB_DIR");
    let source = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap()).join("native/core");
    let mut source_hash = DefaultHasher::new();
    source.hash(&mut source_hash);
    let build = PathBuf::from(env::var_os("OUT_DIR").unwrap())
        .join(format!("core-build-{:016x}", source_hash.finish()));
    let status = Command::new("cmake")
        .args(["-S"])
        .arg(&source)
        .arg("-B")
        .arg(&build)
        .arg(if env::var_os("CARGO_FEATURE_NVENC").is_some() {
            "-DHORUS_ENABLE_NVENC=ON"
        } else {
            "-DHORUS_ENABLE_NVENC=OFF"
        })
        .arg("-DCMAKE_BUILD_TYPE=Release")
        .status()
        .expect("CMake is required for native Teach support");
    assert!(status.success(), "Native core configuration failed");
    let status = Command::new("cmake")
        .arg("--build")
        .arg(&build)
        .args(["--parallel", "2"])
        .status()
        .expect("Failed to build native core");
    assert!(status.success(), "Native core build failed");
    println!("cargo:rustc-link-search=native={}", build.display());
    for name in [
        "static=horus_native_core",
        "dylib=stdc++",
        "dylib=crypto",
        "dylib=zstd",
        "dylib=pthread",
        "dylib=assimp",
        "dylib=tinyxml2",
        "dylib=z",
        "dylib=png",
        "dylib=jpeg",
        "dylib=webp",
        "dylib=tiff",
        "dylib=dl",
    ] {
        println!("cargo:rustc-link-lib={name}");
    }
    if env::var_os("CARGO_FEATURE_NVENC").is_some() {
        println!("cargo:rustc-link-lib=dylib=avcodec");
        println!("cargo:rustc-link-lib=dylib=avutil");
    }
    if env::var_os("CARGO_FEATURE_ROS2").is_none() {
        return;
    }
    // Deployments may provide an installed ABI-compatible native transport.
    let lib = if let Some(path) = env::var_os("HORUS_NATIVE_ROS_LIB_DIR") {
        PathBuf::from(path)
    } else {
        let source = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap()).join("native");
        // Cargo can reuse OUT_DIR when verifying the extracted crate. A CMake
        // cache belongs to one absolute source path, so isolate it by that path.
        let mut source_hash = DefaultHasher::new();
        source.hash(&mut source_hash);
        let build = PathBuf::from(env::var_os("OUT_DIR").unwrap())
            .join(format!("native-build-{:016x}", source_hash.finish()));
        let status = Command::new("cmake")
            .arg("-S")
            .arg(source)
            .arg("-B")
            .arg(&build)
            .args([
                "-DCMAKE_BUILD_TYPE=Release",
                "-DPython3_EXECUTABLE=/usr/bin/python3",
            ])
            .status()
            .expect("CMake is required for the ros2 feature");
        assert!(
            status.success(),
            "Source your ROS 2 environment before enabling the ros2 feature"
        );
        let status = Command::new("cmake")
            .arg("--build")
            .arg(&build)
            .args(["--parallel", "2"])
            .status()
            .expect("Failed to build native ROS transport");
        assert!(status.success(), "Native ROS transport build failed");
        build
    };
    println!("cargo:rustc-link-search=native={}", lib.display());
    println!("cargo:rustc-link-lib=dylib=horus_native_ros");
    // Examples/binaries built here can run without a source-tree LD_LIBRARY_PATH.
    // Installed downstream applications must install the transport beside ROS libraries.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib.display());
}
