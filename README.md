## Install and build
*Tested and built on Windows 11 22H2 using Visual Studio 2022*

### JoltPhysics
- Clone Jolt and prepare the VS2022 solution
```
git clone https://github.com/jrouwe/JoltPhysics.git
cd .\JoltPhysics\Build
.\cmake_vs2022_cl.bat
```
- Open the Jolt solution and select the following projects in the solution explorer:
    - HelloWorld
    - Jolt
    - JoltViewer
    - PerformanceTest
    - Samples
    - TestFramework
    - UnitTests
- Right click and open properties
- In `Configuration Properties > C/C++ > Code Generation > Runtime Library`, select `Multi-threaded Debug DLL` and click apply.
- Build the solution

### Raylib
- Download [Raylib static lib](https://github.com/raysan5/raylib/releases/download/4.5.0/raylib-4.5.0_win64_msvc16.zip) and unzip its contents in a folder named `raylib`

### STB
- Clone stb
```
git clone https://github.com/nothings/stb.git
```

### Nvidia Falcor
- Clone Falcor and setup the VS2022 solution
```
git clone https://github.com/NVIDIAGameWorks/Falcor.git
cd .\Falcor\
.\setup_vs2022.bat
```
- Open the solution found in `.\Falcor\build\windows-vs2022` and build the whole solution

