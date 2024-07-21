
# Real-Time Ray Tracing In OpenGL 

Implementation of a real-time ray-tracing renderer that handles global illumination.



## Getting Started

### Dependencies
- **GLFW**: OpenGL/Vulkan API for creating windows, contexts and surfaces, receiving input and events. To manually install GLFW:
```bash
$ git clone https://github.com/glfw/glfw.git
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```
- **Glad**. Generates a loader for OpenGL or Vulkan that dynamically loads functions at runtime based on the specifications and extensions supported by your system's graphics driver. The source and header files are already provided, but can be obtained from https://glad.dav1d.de/ 
- **cglm**: Highly optimized 2D|3D math library provides lot of utils to help math operations to be fast and quick to write. To manually install cglm:
```bash
$ git clone https://github.com/glfw/glfw.git
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```
- **tinyobjloader-c**: Header-only library for loading wavefront.obj 3D models. The header file is already provided, but can be obtained from https://github.com/syoyo/tinyobjloader-c

### Build
The project is able to generate binaries for both Linux distros and Windows. 

#### Linux

For the listed dependencies, only the GLFW static library needs to be compiled according to your distro. It can be compiled from the source code or installed using your distro's package manager. For example, in Arch or Ubuntu:
```
// Arch
sudo pacman -S glfw

// Ubuntu
sudo apt install libglfw3-dev
```
The project is built using ``` cmake ``` and ``` make ```, however a bash script ``` build.sh ```  is provided to simplify the build process.


#### Windows

A pre-compiled GLFW static library (version 3.4) is provided. Additionally, you need to install MinGW with an up-to-date C compiler. The original MinGW project available at SourceForge is no longer maintained. Instead, a fork available at winlibs.com was used. The tested version is **GCC 14.1.0 (with POSIX threads) + LLVM/Clang/LLD/LLDB 18.1.8 + MinGW-w64 12.0.0 (UCRT) - release 3**. After downloading, add the path to the ```bin``` folder to the ```PATH``` system environment variable. To build the project, run the provided batch script ```build.bat```.

### Self-contained examples
The project includes self-contained examples that can be built individually. Each example demonstrates specific ray tracing concepts, such as a ray traced sphere or triangle. These examples can be built with either build.sh or build.bat, depending on your OS.
 
