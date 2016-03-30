# algorithm-simulation
A simulation of the data that the mapping robot will be generating. Used to test the map-building algorithm

### Dependencies
This project uses SDL 2.0.4: https://www.libsdl.org/

### Setup
In order to use SDL, the following steps must be taken:
* Download v2.0.4 of SDL (above)
* Ensure that the headers are in your include path
 * On Visual Studio: Project Properties -> Configuration Properties -> VC++ Directories -> Include Directories. Add SDL2-2.0.4/include
* Ensure that the library directories are properly added
 * On VS: Project Properties -> Configuration Properties -> VC++ Directories -> Library Directories. Add SDL2-2.0.4/lib/x86
* Ensure that the linker knows which library files to include
 * On VS: Project Properties -> Linker -> Input -> Additional Dependencies. Add SDL2.lib and SDL2main.lib
* Copy the dynamic library file into the base directory of the project. It's found at SDL/lib/x86/SDL2.dll
