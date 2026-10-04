from conan import ConanFile
from conan.tools.files import copy
import os

class LocalMavlinkConan(ConanFile):
    name = "mavlink"
    version = "2.0"
    package_type = "header-library"

    # Use a tuple to force Conan to recursively capture everything in c_library_v2
    exports_sources = ("c_library_v2/*",)

    def package(self):
        # Path where c_library_v2 was exported in the build context
        src_dir = os.path.join(self.folders.base_source, "c_library_v2")

        # Target path inside package_folder
        dst_dir = os.path.join(self.package_folder, "c_library_v2")

        # Copy all headers and dialect folders into package_folder/c_library_v2
        copy(self, "*", src=src_dir, dst=dst_dir)

    def package_info(self):
        self.cpp_info.includedirs = ["c_library_v2"]