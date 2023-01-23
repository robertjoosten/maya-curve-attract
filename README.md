# maya-curve-attract
Maya curve attract node will pull the vertices of the meshes towards the 
closest point on the curve based on the `distance`, `parameter` and `falloff` 
attributes. By default, the points are displayed along the normal of the 
vertex but this can be changed to use the vector of the vertex to the closest 
point on the curve using the `useNormal` attribute. 

## Installation
* Extract the content of the .rar file anywhere on disk.
* Build the plugin for a specific Maya version.
* Drag the curve-attract.mel file in Maya to permanently install the script.

## Compiling
Building the plugin using cmake will place the plugin in the plug-ins folder 
with a maya version divider. This will ensure the plug-in is compatible with 
the launched version of Maya.

1. Open Terminal
```
cd <PATH_TO_MODULE>
mkdir build/<MAYA_VERSION>
cd build/<MAYA_VERSION>
cmake -A x64 -T v141 -DMAYA_VERSION=<MAYA_VERSION> ../../
cmake --build . --target install --config Release
```

## Usage
Once the plug-in is build and loaded a new `curveAttract` node can be 
created by selecting the meshes and running the 
`cmds.deformer(type="curveAttract")` command. 

<p align="center"><img src="icons/curve-attract-network-example.png?raw=true"></p>

After this the curve needs to be manually connected into the `inputCurve` 
attribute on the node.

<p align="center"><img src="icons/curve-attract-scene-example.png?raw=true"></p>

The `parameter` ramp is used to generate multiplier values for the parameter
of the closest point on the curve. This can come in handy to generate a zipper
effect. The `falloff` ramp is used to generate multiplier values for the 
falloff allowing for a smoother transition.

<p align="center"><img src="icons/curve-attract-attribute-example.png?raw=true"></p>

As the dot product is used to calculate the distance, smoothing 
operations are in place to smooth vertices where no displacement is
calculated because of the dot product. For this the `smoothingStep`
and `smoothingIterations` attributes can be used.