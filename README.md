# Ray-Tracing-Engine

This repository contains the code and report for the final project of the CSE2215 - Computer Graphics course at TU Delft. The goal of the project is to implement a fully functional ray tracer with various features, such as acceleration data structure, shading models, recursive ray reflections, transparency, interpolation, texture mapping, lights and shadows, multisampling, and some extra features.

## Implemented Features:
- Acceleration Structure: Bounding Volume Hierarchy
- Shading Models (Blinn-Phong, Lambertian)
- Recursive Ray Reflection and Refractions
- Normal Interpolation for surface smoothing
- Texture Mapping
- Lighting and Shadows
- Multi Sampling Anti-Aliasing (MSAA)
- Motion Blur
- Depth of Field
- Bloom effect
- Advanced Glossy Reflections

## How to run
HIGHLY RECOMMENDED TO RUN IN RELEASE MODE FOR QUICK EXECUTION
To run the project, you need to compile the code with CMake and Visual Studio 2022 on Windows. You can use the following CMake argument to use your own intersection functions instead of the prebuilt library: -DUSE_PREBUILT_INTERSECT=OFF. After compiling, you can launch the application and choose from a selection of test scenes. You can also load your own scene by overwriting the custom.obj file in the data folder. You can switch between rasterization and ray tracing modes, and enable or disable different features from the UI. You can also render an image and save it to a file by clicking the “Render to file” button.
