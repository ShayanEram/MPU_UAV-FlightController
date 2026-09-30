from conan import ConanFile
import os

class LocalMavlinkConan(ConanFile):
    name = "mavlink"
    version = "2.0"
    package_type = "header-library"

    def layout(self):
        # Point directly to the neighboring git submodule directory on your disk
        self.folders.source = "c_library_v2"
        # Map the include directories directly to the source folder
        self.cpp_info.includedirs = ["."]