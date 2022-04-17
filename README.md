# EC527_Project

## Important Dates  

### During the week of 4/19-4/22  
Second meeting with instructor. By this time you should understand your problem thoroughly, have updated your serial code, and started work on your parallel and GPU versions (if you are doing them).  
***DELIVERABLE:** You should prepare an informal document (slides are fine) that describe the architectures you are supporting and plans for partitioning.*    

### During the week of 4/25-4/19  
Meetings with instructor to check progress/problems.  

### Presentation (be ready by Monday 5/2)
A full presentation (8-15 minutes, depending on the size of the group) describing the bulk of the work. There are two presentation days, 5/2 & 5/4. You should be prepared to go on either of the days.  
***DELIVERABLE:** Slides due by 16:00 on **5/1**.*  
<details>
  <summary>Presentation Guidelines</summary>
  
  The talk should be high quality and well-prepared. It’s OK if you haven’t completely finished, but you should be substantially done. Giving an 8-15 minute presentation on a problem that most of the audience is not familiar with is challenging and will take real work to make coherent. Larger groups get more time. In any 
case, your talk should include (most of) the following:
* Description of the problem
* What the serial code/algorithm looks like. What is the algorithm? What is the complexity?
* Where does the time go? What is the arithmetic intensity? 
* What are the primary data structures? What is the memory reference pattern?
* Have you modified the algorithm to run in parallel (or which parallel algorithm you selected if there is a choice)?
* For the parallel (and GPU) parts, how were the data and computations partitioned?
* Overview of your optimized codes. What are the optimizations? What problems did you have? 
* Experiments and results. What worked?
* A couple minutes for some brief Q&A
  </details>
 
### Report due Tuesday 5/10 @ Noon  
Final write-up submitted (incorporating feed-back from presentations)


## Files  
`child_cam_frame.obj` : Mesh file for a small child (~5K vertices).

`child_keypoints.txt` : **Updated 4/11/22 to fix head and torso values** 6 keypoints for the small child mesh (in order: head, torso, right arm, left arm, right leg, left leg). 

`humanoid_tri.obj` : A slightly more complex mesh for development and testing (64 vertices, 96 faces).  

`mesh_keypoint_plot_v2.m` : Matlab script to import the child OBJ file and plot it along with the 6 keypoints (see animated example of the output below).

`obj_to_adjlist.c` : Imports the obj file and builds the adjacency list with distance values; distance values represent distances along edges to adjacent vertices.

~~`obj_to_adjmatrix.c` : Imports the obj file and builds the adjacency matrix (with 6 additional rows and columns for keypoints); values represent distances along edges to adjacent vertices (-1 means no edge exists).~~

`pyramid.obj` : Simple mesh file for a tetrahedron (4 total vertices) for development and testing.

`readObj.m` : Matlab script to import OBJ files.


## Visualizations

### Child OBJ and Keypoints Plotted in Matlab (LOW QUALITY)
Matlab exports cause the jumpiness in the animation.  

![animated_mesh_kp_lowqual.gif](animated_mesh_kp_lowqual.gif)
