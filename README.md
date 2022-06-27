# EC527: High Performance Programming with Multicore and GPUs Project

Our overall goal is to divide a whole human body by several given key points illustrating the main part of the body, such as head, torso, arms, legs and so on. This segment function can be prepared for future use such as training the network. Upon confirming the segment, we use different colors to differentiate the body part, and output them in another obj file. Our pipeline consists of the following parts:

1. Loading data: loading faces and vertices from mesh file
2. Graph building: build the adjacency list which is used to save the distance between each vertex
3. Bellman-Ford to calculate the shortest path from each keypoint to all the other vertices
4. Generating segmented mesh from result

We optimized the step 2 and step 3 by using 1) loop transformation , 2) CPU multithreading ,3) GPU CUDA, and we compared the results. The results we get from the best optimization are shown in a table below.


## Results

| Task                                                                              | Timing Before | Timing Optimized |
| --------------------------------------------------------------------------------- | ------------- | ---------------- |
| Loading data                                                                      | 18.77         | 18.77            |
| **Generate adjacency list graph (link verts and calc edge distances)**            | 2.89          | 0.89             |
| **Bellman-Ford calculate shortest path from each keypoint to all other vertices** | 3753.64       | 150.15           |
| Generating segmentation from result                                               | 0.07          | 0.07             |
| Total Time                                                                        | 3775.37       | 169.88           |

## Visualizations

### Child OBJ and Keypoints Plotted in Matlab (LOW QUALITY)
Matlab exports cause the jumpiness in the animation.  

![animated_mesh_kp_lowqual.gif](animated_mesh_kp_lowqual.gif)

## Files  
`child_cam_frame.obj` : **Updated 4/20/22 to make mesh watertight** Mesh file for a small child (~5K vertices).

`child_keypoints.txt` : **Updated 4/11/22 to fix head and torso values** 6 keypoints for the small child mesh (in order: head, torso, right arm, left arm, right leg, left leg). 

`humanoid_tri.obj` : A slightly more complex mesh for development and testing (64 vertices, 96 faces).  

`mesh_keypoint_plot_v2.m` : Matlab script to import the child OBJ file and plot it along with the 6 keypoints (see animated example of the output below).

`model.mtl` : File with colors specified; required to visualize final obj output file.   

`obj_to_adjlist.c` : Imports the obj file and builds the adjacency list with distance values; distance values represent distances along edges to adjacent vertices.

~~`obj_to_adjmatrix.c` : Imports the obj file and builds the adjacency matrix (with 6 additional rows and columns for keypoints); values represent distances along edges to adjacent vertices (-1 means no edge exists).~~

`pyramid.obj` : Simple mesh file for a tetrahedron (4 total vertices) for development and testing.

`readObj.m` : Matlab script to import OBJ files.
