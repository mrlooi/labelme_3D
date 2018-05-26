This pipeline gets cuboids (box pose and dimensions) from a scene, converts points inside the cuboid to a random color, and projects points to 2D based on different viewpoint poses.

1) Read cuboid poses from json file
2) Get all points inside the cuboid poses (should account for inaccuracies in cuboid pose e.g. expanding the cuboid pose slightly)
3) Assign each cuboid pose pointcloud with a color
4) Generate different viewpoints of the scene (based on pcl viewer)
5) Project each viewpoint pose to point-to-pixel image (with pinhole camera projection, see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html). For each viewpoint pose, generate 2 images: the scene with color annotations (annotated image) and without color (original image). Store the 2D-3D uv map
