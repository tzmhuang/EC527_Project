#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Calculate the distances between each of the 3 vertices that make up the face and add the distances to the adjacency matrix
// Inputs f1, f2, f3 represent the 3 vertices that make up the face
// Calculate distance between f1,f2 then f1,f3 then f2,f3
// Store each calculated distance in adjacency matrix in positions [f1,f2],[f2,f1] then [f1,f3],[f3,f1] then [f2,f3],[f3,f2]
void addEdge(double *verts, double *adjMat, int f1, int f2, int f3, int Ve) {
  double d;
  int f1v = (f1-1)*3;
  int f2v = (f2-1)*3;
  int f3v = (f3-1)*3;
  double f1vX = verts[f1v], f1vY = verts[f1v+1], f1vZ = verts[f1v+2];
  double f2vX = verts[f2v], f2vY = verts[f2v+1], f2vZ = verts[f2v+2];
  double f3vX = verts[f3v], f3vY = verts[f3v+1], f3vZ = verts[f3v+2];

  // f1, f2
  d = sqrt(pow((f2vX - f1vX),2) + pow((f2vY - f1vY),2) + pow((f2vZ - f1vZ),2));
  adjMat[(f1-1)*Ve + (f2-1)] = d;
  adjMat[(f2-1)*Ve + (f1-1)] = d;

  // f1, f3
  d = sqrt(pow((f3vX - f1vX),2) + pow((f3vY - f1vY),2) + pow((f3vZ - f1vZ),2));
  adjMat[(f1-1)*Ve + (f3-1)] = d;
  adjMat[(f3-1)*Ve + (f1-1)] = d;

  // f2, f3
  d = sqrt(pow((f3vX - f2vX),2) + pow((f3vY - f2vY),2) + pow((f3vZ - f2vZ),2));
  adjMat[(f2-1)*Ve + (f3-1)] = d;
  adjMat[(f3-1)*Ve + (f2-1)] = d;

  // printf("In addEdge, f1 (x,y,z), f2 (x,y,z), f3 (x,y,z): \n%d (%.16f, %.16f, %.16f),\n%d (%.16f, %.16f, %.16f)\n%d (%.16f, %.16f, %.16f)\n", f1, f1vX, f1vY, f1vZ, f2, f2vX, f2vY, f2vZ, f3, f3vX, f3vY, f3vZ);
}

// Print the matrix
void printAdjMatrix(double *adjMat, int Ve) {
  int i, j;
  // FILE * fp1;

  // fp1 = fopen ("file.txt", "w+");

  for (i = 0; i < Ve; i++) {
    // fprintf(fp1, "%d: ", i);
    printf("%d: ", i);
    for (j = 0; j < Ve; j++) {
      // fprintf(fp1, "%.16f ", adjMat[i*Ve+j]);
      printf("%.16f ", adjMat[i*Ve+j]);
    }
    // fprintf(fp1, "\n");
    printf("\n");
  }
  // fclose(fp1);
}

int main() {
  FILE * fp;
  char * line = NULL;
  size_t len = 0;
  ssize_t read;
  int Ve = 0, F = 0, j = 0, k = 0, ret;
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

  printf("Ve: %d, F: %d\n", Ve, F);

  // Allocate the matrices  
  double* verts = (double*) malloc(Ve * 3 * sizeof(double));  // Vertices
  double* adjMat = (double*) malloc(Ve * Ve * sizeof(double));  // Adjacency matrix
  int* faces = (int*) malloc(F * 3 * sizeof(int));  // Faces

  // Initialize the matrices to zero
  for (int i = 0; i< Ve*3; i++) verts[i]=0.0;
  for (int i = 0; i< Ve*Ve; i++) adjMat[i]=0.0;
  for (int i = 0; i< F*3; i++) faces[i]=0;

  // Rewind file to beginning and read in all lines again, this time into the allocated matrices
  rewind(fp);
  while ((read = getline(&line, &len, fp)) != -1) {
    if (line[0] == 'v') {
      ret = sscanf(line, "%c %lf %lf %lf", &discard, &verts[j], &verts[j+1], &verts[j+2]);
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
  j=0;
  for (int i=0; i<Ve; i++) {
    printf("%d: %.16f, %.16f, %.16f\n", i, verts[j], verts[j+1], verts[j+2]);
    j+=3;
  }

  // // Print array of faces
  k=0;
  for (int i=0; i<F; i++) {
    printf("%d: %d, %d, %d\n", i, faces[k], faces[k+1], faces[k+2]);
    k+=3;
  }

  // Add edges to adjacency matrix
  k = 0;
  for (int i=0; i<F; i++) {
    addEdge(verts, adjMat, faces[k], faces[k+1], faces[k+2], Ve);
    k+=3;
  }

  // Print adjacency matrix
  printAdjMatrix(adjMat, Ve);

  free(verts);
  free(adjMat);
  free(faces);

  return 0;
}