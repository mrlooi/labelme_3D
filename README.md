# LabelMe With PointClouds!
Tool for labelling and editing pointcloud data. Built to work with PCL!

Labelling pointclouds is now as simple as opening up PCL viewer and pressing some buttons.

## Requirements
- PCL
- OpenCV

## Usage
./labelme_3d pcd_file [-s save_file] 

## Commands
- Save: Ctrl + s
- Annotate: a
- Undo: Ctrl + z
- Redo: Ctrl + y

## How to build
`
mkdir build
cd build
cmake ..
make -j4
`
