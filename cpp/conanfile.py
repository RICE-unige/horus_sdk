from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.files import copy
import os


class HorusCppSdkConan(ConanFile):
    name = "horus-cpp-sdk"
    version = "0.1.0"
    license = "Apache-2.0"
    author = "Omotoye S. Adekoya <omotoye.adekoya@edu.unige.it>, RICE Lab <rice@dibris.unige.it>"
    url = "https://github.com/RICE-unige/horus_sdk"
    description = "HORUS Mixed Reality Robot Management SDK - C++ implementation"
    topics = ("robotics", "mixed-reality", "ros2", "meta-quest", "visualization")
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_ros2": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "with_ros2": False,
    }
    exports_sources = "CMakeLists.txt", "Config.cmake.in", "ament_exports.cmake", "package.xml", "src/*", "include/*", "README.md"

    def export_sources(self):
        copy(self, "LICENSE", src=os.path.join(self.recipe_folder, ".."), dst=self.export_sources_folder)
        copy(self, "*", src=os.path.join(self.recipe_folder, "..", "rust", "native"),
             dst=os.path.join(self.export_sources_folder, "native"))

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        # Add ROS 2 dependencies if enabled
        # Note: ROS 2 typically installed system-wide, not via Conan
        self.requires("nlohmann_json/3.11.3")
        self.requires("openssl/[>=3.0 <4]")
        self.requires("zstd/[>=1.5 <2]")
        self.requires("eigen/3.4.0")
        self.requires("assimp/5.3.1")
        self.requires("tinyxml2/10.0.0")
        self.requires("zlib/[>=1.3 <2]")
        self.requires("libpng/[>=1.6 <2]")
        self.requires("libjpeg-turbo/[>=3 <4]")
        self.requires("libwebp/[>=1.3 <2]")
        self.requires("libtiff/[>=4.5 <5]")

    def build_requirements(self):
        # Build dependencies
        pass

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.variables["HORUS_ENABLE_ROS2"] = bool(self.options.with_ros2)
        tc.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
        tc.variables["HORUS_BUILD_TESTS"] = False
        tc.variables["HORUS_BUILD_EXAMPLES"] = False
        tc.variables["HORUS_BUILD_BENCHMARKS"] = False
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        copy(self, "LICENSE", src=self.source_folder, dst=os.path.join(self.package_folder, "licenses"))

    def package_info(self):
        self.cpp_info.libs = ["horus_cpp_sdk", "horus_native_core"]
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.set_property("cmake_file_name", "horus_cpp_sdk")
        self.cpp_info.set_property("cmake_target_name", "horus::horus_cpp_sdk")
        if self.options.with_ros2:
            self.cpp_info.libs.append("horus_native_ros")
            self.cpp_info.defines.append("HORUS_ROS2_ENABLED=1")
        if self.settings.os in ("Linux", "FreeBSD"):
            self.cpp_info.system_libs.append("pthread")
        
        # Set compiler flags
        if self.settings.compiler == "gcc" or self.settings.compiler == "clang":
            self.cpp_info.cxxflags = ["-std=c++20"]
        elif self.settings.compiler == "msvc":
            self.cpp_info.cxxflags = ["/std:c++20"]
        
        # Set defines
        self.cpp_info.defines.append("HORUS_VERSION=\\\"{}\\\"".format(self.version))
