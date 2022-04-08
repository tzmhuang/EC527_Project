#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef float data_t;

// Calculate the distances between each of the 3 vertices that make up the face and add the distances to the adjacency matrix
// Inputs f1, f2, f3 represent the 3 vertices that make up the face
// Calculate distance between f1,f2 then f1,f3 then f2,f3
// Store each calculated distance in adjacency matrix in positions [f1,f2],[f2,f1] then [f1,f3],[f3,f1] then [f2,f3],[f3,f2]
void addEdge(data_t *verts, data_t *adjMat, int f1, int f2, int f3, int Ve) {
  data_t d;
  int f1v = (f1-1)*3;
  int f2v = (f2-1)*3;
  int f3v = (f3-1)*3;
  data_t f1vX = verts[f1v], f1vY = verts[f1v+1], f1vZ = verts[f1v+2];
  data_t f2vX = verts[f2v], f2vY = verts[f2v+1], f2vZ = verts[f2v+2];
  data_t f3vX = verts[f3v], f3vY = verts[f3v+1], f3vZ = verts[f3v+2];

  // f1, f2
  d = pow((f2vX - f1vX),2) + pow((f2vY - f1vY),2) + pow((f2vZ - f1vZ),2);
  adjMat[(f1-1)*Ve + (f2-1)] = d;
  adjMat[(f2-1)*Ve + (f1-1)] = d;

  // f1, f3
  d = pow((f3vX - f1vX),2) + pow((f3vY - f1vY),2) + pow((f3vZ - f1vZ),2);
  adjMat[(f1-1)*Ve + (f3-1)] = d;
  adjMat[(f3-1)*Ve + (f1-1)] = d;

  // f2, f3
  d = pow((f3vX - f2vX),2) + pow((f3vY - f2vY),2) + pow((f3vZ - f2vZ),2);
  adjMat[(f2-1)*Ve + (f3-1)] = d;
  adjMat[(f3-1)*Ve + (f2-1)] = d;

  // printf("In addEdge, f1 (x,y,z), f2 (x,y,z), f3 (x,y,z): \n%d (%.16f, %.16f, %.16f),\n%d (%.16f, %.16f, %.16f)\n%d (%.16f, %.16f, %.16f)\n", f1, f1vX, f1vY, f1vZ, f2, f2vX, f2vY, f2vZ, f3, f3vX, f3vY, f3vZ);
}

// Print the matrix
void printAdjMatrix(data_t *adjMat, int Ve) {
  int i, j;
  printf("\nAdjacency Matrix:\n");
  for (i = 0; i < Ve; i++) {
    printf("%d: ", i);
    for (j = 0; j < Ve; j++) {
      printf("%f ", adjMat[i*Ve+j]);
    }
    printf("\n");
  }
}

void loadKeypoints(FILE* fptr, data_t** point_array, int** type_array, int* Kp){
    int N;
    data_t n;
    if (NULL == fptr) printf("file cannot be opened \n");
    if (!fscanf(fptr, "%d ", &N)){ printf("Error!"); return;}
    
    data_t* data = (data_t *) calloc(N*3, sizeof(data_t));
    int* type = (int *) calloc(N, sizeof(int));
    int k=0;
    for (int i=0; i<N; i++){
      if(!fscanf(fptr, "%d %f %f %f", &type[i], &data[k], &data[k+1], &data[k+2])){ 
        printf("Error!"); 
        return;
      }
      k+=3;
    }
    *Kp = N;
    *point_array = data;
    *type_array = type;
}

void getKVDistance(data_t* kp_array, data_t* v_array, int N_kp, int N_v, data_t** res){
    int DIM = 3;
    data_t d, kp_x, kp_y, kp_z, gp_x, gp_y, gp_z;
    data_t* data = (data_t *) calloc(N_kp*N_v, sizeof(data_t));

    for (int i=0; i<N_v; i++){
        kp_x = v_array[i*DIM];
        kp_y = v_array[i*DIM+1];
        kp_z = v_array[i*DIM+2];
        
        for (int j=0; j<N_kp; j++){
            gp_x = kp_array[j*DIM];
            gp_y = kp_array[j*DIM+1];
            gp_z = kp_array[j*DIM+2];
            data[i*N_kp + j] = (kp_x-gp_x)*(kp_x-gp_x) + (kp_y-gp_y)*(kp_y-gp_y) + (kp_z-gp_z)*(kp_z-gp_z);
        }
    }
    *res = data;
}

int main() {
  FILE * fp;
  char * line = NULL;
  size_t len = 0;
  ssize_t read;
  int Ve = 0, VeKp = 0, F = 0, j = 0, k = 0, ret;
  char discard;

  fp = fopen("pyramid.obj", "r");
  // fp = fopen("child.obj", "r");
  if (fp == NULL)
    printf("ERROR\n");

  // Read in all lines of file and count number of vertices and number of faces (to allocate matrices next)
  while ((read = getline(&line, &len, fp)) != -1) {
    if (line[0] == 'v') {
      Ve++;  // Count of vertices
    } else if (line[0] == 'f') {
      F++;  // Count of faces
    }
  }

  // Extend the adjacency matrix by 6 to accomodate the keypoints
  VeKp = Ve + 6;

  printf("Ve: %d, F: %d\n", Ve, F);

  // Allocate the matrices  
  data_t* verts = (data_t*) malloc(Ve * 3 * sizeof(data_t));  // Vertices
  data_t* adjMat = (data_t*) malloc(VeKp * VeKp * sizeof(data_t));  // Adjacency matrix
  int* faces = (int*) malloc(F * 3 * sizeof(int));  // Faces

  // Initialize the matrices to zero or -1
  for (int i = 0; i< Ve*3; i++) verts[i]=0.0;
  for (int i = 0; i< VeKp*VeKp; i++) adjMat[i]=-1.0;
  for (int i = 0; i< F*3; i++) faces[i]=0;

  // Rewind file to beginning and read in all lines again, this time into the allocated matrices
  rewind(fp);
  while ((read = getline(&line, &len, fp)) != -1) {
    if (line[0] == 'v') {
      ret = sscanf(line, "%c %f %f %f", &discard, &verts[j], &verts[j+1], &verts[j+2]);
      // printf("ret: %d, %s", ret, line);
      j+=3;
    } else if (line[0] == 'f') {
      ret = sscanf(line, "%c %d %d %d", &discard, &faces[k], &faces[k+1], &faces[k+2]);
      // printf("%s", line);
      k+=3;
    }
  }

  // Close the file and clean-up a little
  fclose(fp);
  if (line)
    free(line);

  // Print array of vertices
  printf("\nVertices:\n");
  j=0;
  for (int i=0; i<Ve; i++) {
    printf("%d: %f, %f, %f\n", i, verts[j], verts[j+1], verts[j+2]);
    j+=3;
  }

  // Print array of faces
  printf("\nFaces:\n");
  k=0;
  for (int i=0; i<F; i++) {
    printf("%d: %d, %d, %d\n", i, faces[k], faces[k+1], faces[k+2]);
    k+=3;
  }
  // Add edges to adjacency matrix
  k = 0;
  for (int i=0; i<F; i++) {
    addEdge(verts, adjMat, faces[k], faces[k+1], faces[k+2], VeKp);
    k+=3;
  }

  // Print adjacency matrix
  printAdjMatrix(adjMat, VeKp);


  // Load Keypoints
  data_t* keypoints;
  int* keypoint_types;
  int Kp;
  fp = fopen("kp_example.txt", "r");
  loadKeypoints(fp, &keypoints, &keypoint_types, &Kp);
  printf("\nKeyppoints:\n");
  k = 0;
  for (int i=0; i<Kp; i++){
    printf("%d [type %d]: %f, %f, %f \n", i, keypoint_types[i], keypoints[k], keypoints[k+1], keypoints[k+2]);
    k+=3;
  }
  fclose(fp);

  // Calculate Keypoint-vertex distance
  data_t* kv_distances;
  getKVDistance(keypoints, verts, Kp, Ve, &kv_distances);

  printf("\nKeypoint-Vertex Distances:\n");
  for(int i=0; i<Ve; i++){
    printf("Vt %d: ", i);
      for(int j=0; j<Kp; j++){
          printf("%f ",kv_distances[i*(Kp) + j]);
      }
      printf("\n");
  }
  
  //find closest 1 kp for all vertices

  int min_id;
  data_t min_d, d;
  int* nearest_kp = (int*) malloc(Ve * sizeof(int));
  for(int i=0; i<Ve; i++){
    min_id = -1;
    min_d = 99999;
    for(int j=0; j<Kp; j++){
      d = kv_distances[i*(Kp) + j];
      if(min_d > d){
        min_id = j;
        min_d = d;
      }
    }
    nearest_kp[i] = min_id;
  }
  
  printf("\nClosest KP id:\n");
  for(int i=0; i<Ve; i++){
    printf("Vt %d: ", i);
    printf("%d \n",nearest_kp[i]);
  }


  // Clean-up
  free(verts);
  free(adjMat);
  free(faces);
  free(keypoints);
  free(kv_distances);
  free(keypoint_types);

  return 0;
}