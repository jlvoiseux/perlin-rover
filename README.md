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


### TODO
- Good follow camera
- Sculpt nice terrain
- Nice 3d vehicle
- Prepare materials
- Final scene
- Prepare final repos (Fork Falcor, change Animation controller.cpp:295)

### Some notes about Falcor
- Media paths are set through settings.json
- Copying the binary output of Falcor is mandatory - shaders are looked for in `getRuntimeDirectory() / "shaders"`. See `.\Falcor\Source\Falcor\Core\Platform\OS.cpp:64`

## Third-party licenses:
### JoltPhysics
### Falcor
### STB
### Textures
Created using Gravel032, Plastic012A, Metal009, Metal031 from ambientCG.com,
licensed under the Creative Commons CC0 1.0 Universal License.