# Perlin Rover
This C++ project is an experiment in **heightmap-based terrain generation, physics simulation of a four-wheeled vehicle and
real-time ray tracing using Nvidia RTX**. This is achieved through the combination of [Jolt](https://github.com/jrouwe/JoltPhysics), an open-source real-time physics engine,
and [Falcor](https://github.com/NVIDIAGameWorks/Falcor), an experimental renderer from Nvidia. The result is a simple interactive application where a ray-traced four-wheeled vehicle
can be moved around a procedurally generated landscape.

## Screenshots
<img width="959" alt="Screenshot 2025-02-08 194811" src="https://github.com/user-attachments/assets/4e1e97fe-26dc-48b7-936b-398169abc383" />
<img width="956" alt="Screenshot 2025-02-08 194955" src="https://github.com/user-attachments/assets/195e80f9-bf83-48dc-8f3b-404ecf9121c4" />
![perlin-rover-opt](https://github.com/user-attachments/assets/9338dfb2-abb4-4f30-97cd-d4c38f053e48)

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
- Apply the following patch
```
--- a/Source/Falcor/Scene/Animation/AnimationController.cpp
+++ b/Source/Falcor/Scene/Animation/AnimationController.cpp
@@ -292,7 +292,7 @@ namespace Falcor
         {
             NodeID nodeID = pAnimation->getNodeID();
             FALCOR_ASSERT(nodeID.get() < mLocalMatrices.size());
-            mLocalMatrices[nodeID.get()] = pAnimation->animate(time);
+            //mLocalMatrices[nodeID.get()] = pAnimation->animate(time);
             mMatricesChanged[nodeID.get()] = true;
         }
     }
```
- Open the solution found in `.\Falcor\build\windows-vs2022` and build the whole solution

#### Some notes
- Media paths are set through settings.json
- Copying the binary output of Falcor is mandatory - shaders are looked for in `getRuntimeDirectory() / "shaders"`. See `.\Falcor\Source\Falcor\Core\Platform\OS.cpp:64`

## Dependencies
- Jolt Physics ([source](https://github.com/jrouwe/JoltPhysics), [license](https://github.com/jrouwe/JoltPhysics?tab=MIT-1-ov-file#readme))
- Nvidia Falcor ([source](https://github.com/NVIDIAGameWorks/Falcor), [license](https://github.com/NVIDIAGameWorks/Falcor?tab=License-1-ov-file#readme))
- STB ([source](https://github.com/nothings/stb), [license](https://github.com/nothings/stb?tab=License-1-ov-file#readme))
- Textures: Gravel032, Plastic012A, Metal009, Metal031 from ambientCG.com, licensed under the Creative Commons CC0 1.0 Universal License.
