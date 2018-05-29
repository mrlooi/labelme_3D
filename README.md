# LabelMe With PointClouds!
Tool for labelling and editing pointcloud data. Built to work with PCL!

Labelling pointclouds is now as simple as opening up PCL viewer and pressing some buttons.

## Requirements
- PCL
- OpenCV

## Usage
./labelme_3d pcd_file [-s output_file] 

## Commands
Annotate, delete, and extract points in a cloud by drawing polygons (to form a mask around the desired points)
- Annotate: `a` (annotate points in a polygon - currently assigns a random color to the points)
- Undo-Annotation: `u`  (converts points in the polygon back to their original colors)
- Delete: `d`   (removes all points in the polygon)
- Extract: `x`  (this does the opposite of "delete" i.e. extracts the points in the mask only)
- Undo: `Ctrl + z`
- Redo: `Ctrl + y`
- Save: `Ctrl + s`  (saves to output_file, must be set as an argument with '-s', see Usage)

## How to build
mkdir build  
cd build  
cmake ..  
make -j4  

