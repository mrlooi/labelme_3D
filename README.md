# LabelMe With PointClouds!
Tool for labelling and editing pointcloud data. Built to work with PCL!

Labelling pointclouds is now as simple as opening up PCL viewer and pressing some buttons.

Annotate, delete, and extract points in a cloud by drawing polygons (to form a mask around the desired points)

## Requirements
- PCL
- OpenCV

## Usage
./labelme_3d pcd_file [-s output_file] 

## About
This tool is comprised of 2 components:
- PCL Viewer
- OpenCV pop-up window

PCL viewer is used to visualize the pointcloud from arbitrary camera viewpoints. 
Each time an operation on the pointcloud is about to be performed, a window (OpenCV window) will pop up and display the 2D-projected image generated from the current camera viewpoint.
An operation is usually conducted via drawing polygons on the projected image of the pointcloud. A polygon generates a mask on the image, and the mask is then mapped back onto the 3D points in the pointcloud.
In the OpenCV window, use `left-click` to draw polygon points, `backspace` to undo the previous polygon point, `c` to clear all polygon points, and `q` to exit the window.

## Commands
### PCL Viewer
- Annotate: `a` (annotate points in a polygon - currently assigns a random color to the points)
- Undo-Annotation: `u`  (converts points in the polygon back to their original colors)
- Delete: `d`   (removes all points in the polygon)
- Extract: `x`  (this does the opposite of "delete" i.e. extracts the points in the mask only)
- Undo: `Ctrl + z`
- Redo: `Ctrl + y`
- Save: `Ctrl + s`  (saves to output_file, must be set as an argument with '-s', see Usage)
- Quit: `q`  (quits the program)

### OpenCV pop-up window
- Draw a polygon point: `Left-click`
- Undo previous polygon point: `backspace`
- Clear: `c` (clears all existing polygon points)
- Quit: `q`  (closes the pop-up window (but not the PCL viewer))

## How to build
mkdir build  
cd build  
cmake ..  
make -j4  

