from conan import ConanFile
from conan.tools.gnu import PkgConfig
from conan.tools.system import package_manager

class PigpioConan(ConanFile):
    name = "pigpio"
    version = "1.0"
    settings = "os", "compiler", "build_type", "arch"

    def system_requirements(self):
        apt = package_manager.Apt(self)
        apt.install(["libpigpio-dev"])

    def package_info(self):
        self.cpp_info.libs = ["pigpio", "pthread", "rt"]