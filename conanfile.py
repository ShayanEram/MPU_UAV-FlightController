from conan import ConanFile
from conan.tools.cmake import CMake
import os

PACKAGE_VERSION = "0.1.0"

class UAV_FlightController(ConanFile):
    name = "nw-interface"
    version = PACKAGE_VERSION
    package_type = "application"

    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    requires = [
        "boost/1.86.0",
        "nlohmann_json/3.11.3",
        "yaml-cpp/0.8.0",
        "spdlog/1.14.1",
        "gtest/1.15.0",
        "mavlink/2.0",
        "pigpio/1.0"
    ]

    default_options = {
        "boost/*:header_only": True,
        "spdlog/*:header_only": True,
    }

    def configure(self):
        self.settings.compiler.cppstd = "23"

    def layout(self):
        self.folders.build = "build"
        self.folders.generators = "build/generators"
        self.cpp.source.includedirs = ["include"]
        self.cpp.source.srcdirs = ["src"]

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

if __name__ == '__main__':
    print(PACKAGE_VERSION)
