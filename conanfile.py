from conan import ConanFile

class LiDARProcessorConan(ConanFile):
    name = "lidarprocessor"
    version = "0.1.0"
    license = "MIT"
    description = "Standalone Velodyne-centric LiDAR processor with visualization"
    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "imgui/*:glfw": True,
        "imgui/*:opengl3": True,
    }
    generators = "CMakeDeps", "CMakeToolchain"
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "shaders/*", "data/*"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        self.options["imgui/*"].glfw = True
        self.options["imgui/*"].opengl3 = True

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("glfw/3.4")
        self.requires("glew/2.2.0")
        self.requires("glm/cci.20230113")
        self.requires("imgui/cci.20230105+1.89.2.docking")
        self.requires("opengl/system")

    def build_requirements(self):
        self.tool_requires("cmake/3.30.1")

    def package_info(self):
        self.cpp_info.libs = ["lidarprocessor"]
