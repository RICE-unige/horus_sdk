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
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "README.md"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        # Add ROS 2 dependencies if enabled
        # Note: ROS 2 typically installed system-wide, not via Conan
        pass

    def build_requirements(self):
        # Build dependencies
        pass

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        if self.options.with_ros2:
            tc.variables["ROS2_FOUND"] = True
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
        self.cpp_info.libs = ["horus_cpp_sdk"]
        self.cpp_info.includedirs = ["include"]
        
        # Set compiler flags
        if self.settings.compiler == "gcc" or self.settings.compiler == "clang":
            self.cpp_info.cxxflags = ["-std=c++17"]
        elif self.settings.compiler == "msvc":
            self.cpp_info.cxxflags = ["/std:c++17"]
        
        # Set defines
        self.cpp_info.defines = ["HORUS_VERSION=\\\"{}\\\"".format(self.version)]
