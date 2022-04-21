/*

 gcc -O1 timing.c -lrt -lm -std=gnu11 -o timing

*/

// Adjacency List code borrowed from https://www.geeksforgeeks.org/graph-and-its-representations/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


#define INT_MAX 1e+6
#define NMINV 1

typedef float data_t;

// A structure to represent an adjacency list node
struct AdjListNode
// dest represent the index of the node
// dist is the location number
{
    int dest;
    data_t dist;
    struct AdjListNode *next;
};

// A structure to represent an adjacency list
// the AdjList consists of the head, and a nodeCount of how many nodes (vertices) are in this list
struct AdjList
{
    int nodeCount;
    struct AdjListNode *head;
};

// A structure to represent a graph. A graph is an array of adjacency lists.
// Size of array will be Ve (number of vertices in graph)
struct Graph
{
    int Ve;
    struct AdjList *array;
};

// A utility function to create a new adjacency list node
struct AdjListNode *newAdjListNode(int dest, data_t dist)
{
    struct AdjListNode *newNode = (struct AdjListNode *)malloc(sizeof(struct AdjListNode));
    newNode->dest = dest;
    newNode->dist = dist;
    newNode->next = NULL;
    return newNode;
}

// A utility function that creates a graph of Ve vertices
struct Graph *createGraph(int Ve)
{
    struct Graph *graph = (struct Graph *)malloc(sizeof(struct Graph));
    graph->Ve = Ve;
    // Create an array of adjacency lists.  Size of array will be Ve
    graph->array = (struct AdjList *)malloc(Ve * sizeof(struct AdjList));

    // Initialize each adjacency list as empty by making head as NULL
    int i;
    for (i = 0; i < Ve; ++i)
    {
        graph->array[i].nodeCount = 0;
        graph->array[i].head = NULL;
    }

    return graph;
}

double interval(struct timespec start, struct timespec end)
{
  struct timespec temp;
  temp.tv_sec = end.tv_sec - start.tv_sec;
  temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  if (temp.tv_nsec < 0) {
    temp.tv_sec = temp.tv_sec - 1;
    temp.tv_nsec = temp.tv_nsec + 1000000000;
  }
  return (((double)temp.tv_sec)*1.0e3 + ((double)temp.tv_nsec*1.0e-6));
}
/*
 *      This method does not require adjusting a #define constant
 *
 *        How to use this method:
 *
 *              struct timespec time_start, time_stop;
 *                    clock_gettime(CLOCK_REALTIME, &time_start);
 *                          // DO SOMETHING THAT TAKES TIME
 *                                clock_gettime(CLOCK_REALTIME, &time_stop);
 *                                      measurement = interval(time_start, time_stop);
 *
 *                                       */


/* -=-=-=-=- End of time measurement declarations =-=-=-=- */


// Print the adjacency list
void printGraph(struct Graph *graph)
{
    int v;
    for (v = 0; v < graph->Ve; ++v)
    {
        struct AdjListNode *pCrawl = graph->array[v].head;
        printf("\nAdjacency list of vertex %d (dist.)\n head ", (v + 1));
        while (pCrawl)
        {
            printf("-> %d (%f) ", ((pCrawl->dest) + 1), pCrawl->dist);
            pCrawl = pCrawl->next;
        }
        printf("\n");
    }
}

// Add an edge and its distance to the undirected graph
void addEdge(struct Graph *graph, int src, int dest, data_t dist)
{
    // Used to check whether the node has already been attached to the src or dest (so it won't be added again)
    int dupe = 0;
    // Add an edge from src to dest.
    // A new node is added to the adjacency list of src.  The node is added at the beginning.
    struct AdjListNode *check = NULL;
    struct AdjListNode *newNode = newAdjListNode(dest, dist);

    if (graph->array[src].head == NULL)
    {
        graph->array[src].head = newNode;
        graph->array[src].nodeCount += 1;
    }
    else
    {
        check = graph->array[src].head;
        if (check->dest == dest)
            dupe = 1;
        while (check->next != NULL)
        {
            if (check->dest == dest)
                dupe = 1;
            check = check->next;
        }
        if (!dupe && (check->dest != dest))
        {
            check->next = newNode;
            graph->array[src].nodeCount += 1;
        }
    }

    // Since graph is undirected, add an edge from dest to src also
    newNode = newAdjListNode(src, dist);
    dupe = 0;
    if (graph->array[dest].head == NULL)
    {
        graph->array[dest].head = newNode;
        graph->array[dest].nodeCount += 1;
    }
    else
    {
        check = graph->array[dest].head;
        if (check->dest == src)
            dupe = 1;
        while (check->next != NULL)
        {
            if (check->dest == src)
                dupe = 1;
            check = check->next;
        }
        if (!dupe && (check->dest != src))
        {
            check->next = newNode;
            graph->array[dest].nodeCount += 1;
        }
    }
}

// Calculate the distances between each of the 3 vertices that make up the face and add the distances to the adjacency list via addEdge()
// Inputs f1, f2, f3 represent the 3 vertices that make up the face
// Calculate distance between f1,f2 then f1,f3 then f2,f3
void addEdges(data_t *verts, struct Graph *graph, int f1, int f2, int f3)
{
    data_t d;
    // Create variables to find the vertices in the verts matrix
    int f1v = (f1 - 1) * 3;
    int f2v = (f2 - 1) * 3;
    int f3v = (f3 - 1) * 3;
    // Extract the x,y,z values for each vertex from the verts matrix
    data_t f1vX = verts[f1v], f1vY = verts[f1v + 1], f1vZ = verts[f1v + 2];
    data_t f2vX = verts[f2v], f2vY = verts[f2v + 1], f2vZ = verts[f2v + 2];
    data_t f3vX = verts[f3v], f3vY = verts[f3v + 1], f3vZ = verts[f3v + 2];

    // f1, f2
    d = pow((f2vX - f1vX), 2) + pow((f2vY - f1vY), 2) + pow((f2vZ - f1vZ), 2);
    addEdge(graph, (f1 - 1), (f2 - 1), d);

    // f1, f3
    d = pow((f3vX - f1vX), 2) + pow((f3vY - f1vY), 2) + pow((f3vZ - f1vZ), 2);
    addEdge(graph, (f1 - 1), (f3 - 1), d);

    // f2, f3
    d = pow((f3vX - f2vX), 2) + pow((f3vY - f2vY), 2) + pow((f3vZ - f2vZ), 2);
    addEdge(graph, (f2 - 1), (f3 - 1), d);
}

void PrintDistance(data_t dist[], int n, int source)
{
    // This function prints the final solution
    printf("The result of the BellmanFord and a given vert %d is: ", source);
    printf("\nVertex\tDistance from Source Vertex\n");
    int i;

    for (i = 0; i < n; ++i)
    {
        printf("%d \t\t %f\n", i + 1, dist[i]);
    }
}

void BellmanFord(struct Graph *graph, int index, data_t** distance)
{
    int V = graph->Ve;
    int to;
    struct AdjListNode *from = NULL;
    data_t weight;
    data_t *StoreDistance = (data_t *)calloc(V, sizeof(data_t));
    int i, j, n;
    int source = index - 1;

    // This is initial step that we know , we initialize all distance to infinity except source.
    // We assign source distance as 0(zero)

    for (i = 0; i < V; i++)
        StoreDistance[i] = INT_MAX;

    StoreDistance[source] = 0;

    from = graph->array[source].head;
    while (from != NULL)
    {
        to = from->dest;
        weight = from->dist;
        if (StoreDistance[to] > StoreDistance[source] + weight)
        {
            StoreDistance[to] = StoreDistance[source] + weight;
        }
        from = from->next;
    }

    for (n = 0; n < V - 1; n++)
    {
        for (i = 0; i < V; i++)
        {
            from = graph->array[i].head;
            while (from != NULL)
            {
                to = from->dest;
                weight = from->dist;
                if (StoreDistance[to] > StoreDistance[i] + weight)
                {
                    StoreDistance[to] = StoreDistance[i] + weight;
                }
                from = from->next;
            }
        }
    }

    *distance = StoreDistance;
    return;
}

void loadKeypoints(FILE *fptr, data_t **point_array, int **type_array, int *Kp)
{
    int N;
    data_t n;
    if (NULL == fptr)
        printf("file cannot be opened \n");
    // N represent the whole key-point numbers
    // type_array represent the type, point_array represent the data points
    if (!fscanf(fptr, "%d ", &N))
    {
        printf("Error!");
        return;
    }

    data_t *data = (data_t *)calloc(N * 3, sizeof(data_t));
    int *type = (int *)calloc(N, sizeof(int));
    int k = 0;
    for (int i = 0; i < N; i++)
    {
        if (!fscanf(fptr, "%d %f %f %f", &type[i], &data[k], &data[k + 1], &data[k + 2]))
        {
            printf("Error!");
            return;
        }
        k += 3;
    }
    *Kp = N;
    *point_array = data;
    *type_array = type;
}

void getKVDistance(data_t *kp_array, data_t *v_array, int N_kp, int N_v, data_t **res)
{
    int DIM = 3;
    data_t d, kp_x, kp_y, kp_z, gp_x, gp_y, gp_z;
    data_t *data = (data_t *)calloc(N_kp * N_v, sizeof(data_t));

    for (int i = 0; i < N_kp; i++)
    {
        kp_x = kp_array[i * DIM];
        kp_y = kp_array[i * DIM + 1];
        kp_z = kp_array[i * DIM + 2];

        for (int j = 0; j < N_v; j++)
        {
            gp_x = v_array[j * DIM];
            gp_y = v_array[j * DIM + 1];
            gp_z = v_array[j * DIM + 2];
            // return the smallest distance between kp and v
            data[i * N_v + j] = (kp_x - gp_x) * (kp_x - gp_x) + (kp_y - gp_y) * (kp_y - gp_y) + (kp_z - gp_z) * (kp_z - gp_z);
        }
    }
    *res = data;
}

void faces_kp(int *face_nearest_kp, int *faces, int *nearest_kp, int F){
    int i;
    int k = 0;
    int keyIndex1, keyIndex2, keyIndex3 = 0;
    for (i = 0; i < F; i++)
    {
        keyIndex1 = nearest_kp[faces[k] - 1];
        keyIndex2 = nearest_kp[faces[k + 1] - 1];
        keyIndex3 = nearest_kp[faces[k + 2] - 1];
        if(keyIndex1 == keyIndex2){
            face_nearest_kp[i] = keyIndex1;
        }
        else if(keyIndex1 == keyIndex3){
            face_nearest_kp[i] = keyIndex1;
        }
        else if(keyIndex2 == keyIndex3){
            face_nearest_kp[i] = keyIndex2;
        }
        else{
            face_nearest_kp[i] = keyIndex1;
        }
        k += 3;
    }
}

// Prints the number of nodes attached to each node (i.e., vertices attached to each vertex)
void print_node_counts(struct Graph *graph) {
    int *nodesCount = (int *)malloc(100*sizeof(int));
    int v, c;

    // Init nodesCount to all 0s
    for (int n = 0; n < 100; n++) {
        nodesCount[n] = 0;
    }

    // Pull all counts from graph
    for (v = 0; v < graph->Ve; ++v)
    {
        c = graph->array[v].nodeCount;
        nodesCount[c] += 1;
    }

    // Print all counts
    for (int n=0; n<100 ; n++) {
        if (nodesCount[n] > 0) {
            printf("\nNode count %d, number of nodes: %d\n", n, nodesCount[n]);
        }
    }
}

void output_obj(data_t *verts, int *face_nearest_kp, int *faces, int Ve, int F) {
    char *filename = "segmented_child.obj";
    char seg_color[6][10] = {
                         "Blue",    // head
                         "Red",     // torso
                         "Green",   // right arm
                         "Orange",  // left arm
                         "Purple",  // right leg
                         "Yellow"   // left leg
                     };

    // open the file for writing
    FILE *fp = fopen(filename, "w");
    if (fp == NULL)
    {
        printf("Error opening the file %s", filename);
    }

    // Write headers
    fprintf(fp, "mtllib model.mtl\no segmented_child_mesh\n");
    
    // Write vertices
    int j = 0;
    for (int i = 0; i < Ve; i++) {
        fprintf(fp, "v %f %f %f\n", verts[j], verts[j+1], verts[j+2]);
        j+=3;
    }

    // Write faces
    int currface = -1;
    j = 0;
    for (int i = 0; i < F; i++) {
        if (face_nearest_kp[i] != currface) {
            fprintf(fp, "usemtl %s\n", seg_color[(face_nearest_kp[i])]);
            currface = face_nearest_kp[i];    
        }
        fprintf(fp, "f %d %d %d\n", faces[j], faces[j+1], faces[j+2]);
        j+=3;
    }

    // close the file
    fclose(fp);
}

int main()
{
    FILE *fp;
    char *line = NULL;
    size_t len = 0;
    size_t read;
    int Ve = 0, F = 0, j = 0, k = 0, ret;
    char discard;
    struct timespec time_start, time_stop;
    double elapse_time;


    /**********************************************************************************************
    Step1: Loading the obj file and get the vertices and faces
    */

    clock_gettime(CLOCK_REALTIME, &time_start);
    // fp = fopen("pyramid.obj", "r");
    fp = fopen("child_cam_frame.obj", "r");
    if (fp == NULL)
        printf("ERROR\n");

    // Read in all lines of file and count number of vertices and number of faces (to allocate matrices next)
    while ((read = getline(&line, &len, fp)) != -1)
    {
        if (line[0] == 'v')
        {
            Ve++; // Count of vertices
        }
        else if (line[0] == 'f')
        {
            F++; // Count of faces
        }
    }

    printf("Ve: %d, F: %d\n", Ve, F);

    // Allocate the matrices and create the graph
    data_t *verts = (data_t *)malloc(Ve * 3 * sizeof(data_t)); // Vertices
    int *faces = (int *)malloc(F * 3 * sizeof(int));           // Faces

    // Initialize the matrices to zero
    for (int i = 0; i < Ve * 3; i++)
        verts[i] = 0.0;
    for (int i = 0; i < F * 3; i++)
        faces[i] = 0;

    // Rewind file to beginning and read in all lines again, this time into the allocated matrices
    rewind(fp);
    while ((read = getline(&line, &len, fp)) != -1)
    {
        if (line[0] == 'v')
        {
            ret = sscanf(line, "%c %f %f %f", &discard, &verts[j], &verts[j + 1], &verts[j + 2]);
            // printf("ret: %d, %s", ret, line);
            j += 3;
        }
        else if (line[0] == 'f')
        {
            ret = sscanf(line, "%c %d %d %d", &discard, &faces[k], &faces[k + 1], &faces[k + 2]);
            // printf("%s", line);
            k += 3;
        }
    }

    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for loading the vertices and faces: %f(msec)\n", (float)(elapse_time));


    /***********************************************************************************************
    Step2: Loading the keypoints
    */

    clock_gettime(CLOCK_REALTIME, &time_start);
    // Load Keypoints
    data_t *keypoints;
    int *keypoint_types;
    int Kp;
    fp = fopen("child_keypoints.txt", "r");
    loadKeypoints(fp, &keypoints, &keypoint_types, &Kp);
    // printf("\nKeyppoints:\n");
    k = 0;
    for (int i = 0; i < Kp; i++)
    {
        // printf("%d [type %d]: %f, %f, %f \n", i, keypoint_types[i], keypoints[k], keypoints[k + 1], keypoints[k + 2]);
        k += 3;
    }

    // Close the file and clean-up a little
    fclose(fp);
    if (line)
        free(line);

    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for loading the keypoints: %f(msec)\n", (float)(elapse_time));

    // Create Graph
    struct Graph *graph = createGraph(Ve + Kp);

    // Print array of vertices
    // printf("\nVertices:\n");
    // j = 0;
    // for (int i = 0; i < Ve; i++)
    // {
    //     printf("%d: %f, %f, %f\n", i, verts[j], verts[j + 1], verts[j + 2]);
    //     j += 3;
    // }

    // // Print array of faces
    // printf("\nFaces:\n");
    // k = 0;
    // for (int i = 0; i < F; i++)
    // {
    //     printf("%d: %d, %d, %d\n", i, faces[k], faces[k + 1], faces[k + 2]);
    //     k += 3;
    // }

    /***********************************************************************************************
    Step3: Adding Edges from faces and vertices to graph
    */

    clock_gettime(CLOCK_REALTIME, &time_start);
    // Add edges to adjacency list
    k = 0;
    for (int i = 0; i < F; i++)
    {
        addEdges(verts, graph, faces[k], faces[k + 1], faces[k + 2]);
        k += 3;
    }

    // Print adjacency list
    // printGraph(graph);
    printf("\n");

    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for adding ddges from to graph: %f(msec)\n", (float)(elapse_time));

    /***********************************************************************************************
    Step4: Calculating distance from keypoints to vertexs
    */

    clock_gettime(CLOCK_REALTIME, &time_start);

    // Calculate Keypoint-vertex distance
    data_t *kv_distances;
    getKVDistance(keypoints, verts, Kp, Ve, &kv_distances);

    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for calculating distance from keypoints to vertexs: %f(msec)\n", (float)(elapse_time));

    // printf("\nKeypoint-Vertex Distances:\n");
    // for (int i = 0; i < Kp; i++)
    // {
    //     printf("Kp %d: ", i);
    //     for (int j = 0; j < Ve; j++)
    //     {
    //         printf("%f ", kv_distances[i * (Ve) + j]);
    //     }
    //     printf("\n");
    // }

    /***********************************************************************************************
    Step5: Calculating closet n vertex for all keypoints and add the edges to the graph
    */

    clock_gettime(CLOCK_REALTIME, &time_start);

    // find closest NMINV vertex for all keypoints
    int *nearest_vt = (int *)malloc(Kp * NMINV * sizeof(int));
    { // begin scope
    int min_id, last_min_id;
    data_t min_d, d, last_min;

    // Ugly loops: looping through all keypoint NMINV times
    // Complexity: O(Ve*Kp*NMINV)
    for (int i = 0; i < Kp; i++)
    {
        last_min = -1;
        for (int k = 0; k < NMINV; k++)
        {
            min_d = INT_MAX;
            min_id = -1;
            for (int j = 0; j < Ve; j++)
            {
                d = kv_distances[i * (Ve) + j];
                if (min_d > d && last_min < d)
                {
                    min_id = j;
                    min_d = d;
                }
            }
            nearest_vt[i * NMINV + k] = min_id;
            last_min = min_d;
        }
    }
    } //end scope

    // printf("\nClosest vertex id:\n");
    // for (int i = 0; i < Kp; i++)
    // {
    //     printf("Kp %d: ", i);
    //     for (int j = 0; j < NMINV; j++)
    //     {
    //         printf("%d ", nearest_vt[i * NMINV + j]);
    //     }
    //     printf("\n");
    // }

    // Link keypoints to NMINV vertices
    for (int i = 0; i < Kp; i++)
    {
        for (int j = 0; j < NMINV; j++)
        {
            addEdge(graph, Ve + i, nearest_vt[i * NMINV + j], 0); //[Add edge here]
        }
    }

    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for adding n closet vertexs to its keypoint to graph: %f(msec)\n", (float)(elapse_time));

    // printGraph(graph);
    printf("\n");

    // Print node counts
    // print_node_counts(graph);

    /***********************************************************************************************
    Step6: Run Bellman-Ford for each keypoint
    */

    clock_gettime(CLOCK_REALTIME, &time_start);

    // For each keypoint, run Bellman-Ford
    // Complexity: O(|E|*|K|*|V|)
    // Keypoint id = [Ve, Ve+Kp)
    data_t* dist_array;
    data_t* vk_shortest_dist = (data_t *)calloc(Ve*Kp, sizeof(data_t)); // Ve-by-Kp array
    for (int k=Ve; k<Ve+Kp; k++){
        BellmanFord(graph, k+1, &dist_array); // id starts from 1
        // PrintDistance(dist_array, Ve+Kp, k+1);
        for(int v=0; v<Ve;v++){
            vk_shortest_dist[v*Kp + k-Ve] = dist_array[v]; // copy result into array (alt.: use ptr)
        }
    }

    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for Bellman-Ford for each keypoint: %f(msec)\n", (float)(elapse_time));

    // printf("\nClosest Dist. from Vt to Kp\n");
    // for(int i=0; i<Ve; i++){
    //     printf("Vt %d: ", i);
    //     for(int j=0; j<Kp; j++){
    //         printf("%f ",vk_shortest_dist[i*Kp + j]);
    //     }
    //     printf("\n");
    // }

    /***********************************************************************************************
    Step7: Find closest Kp for all Vt
    */

    clock_gettime(CLOCK_REALTIME, &time_start);
    // Find closest Kp for all Vt
    int *nearest_kp = (int *)malloc(Ve * sizeof(int));
    data_t min_d, d;
    int min_id;
    for(int i=0; i<Ve; i++){
        min_d = INT_MAX;
        min_id=0;
        for(int j=0; j<Kp; j++){
            d = vk_shortest_dist[i*Kp + j];
            if (d < min_d){
                min_d = d;
                min_id = j;
            }
        }
        nearest_kp[i] = min_id;
    }


    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for Find closest Kp for all Vts: %f(msec)\n", (float)(elapse_time));

    // printf("\nClosest Kp id:\n");

    // for (int i = 0; i < Ve; i++)
    // {
    //     printf("Vt %d: %d \n", i, nearest_kp[i]);
    // }


    /***********************************************************************************************
    Step8: Find closest Kp for all Faces
    */

    clock_gettime(CLOCK_REALTIME, &time_start);

    int *face_nearest_kp = (int *)malloc(F * sizeof(int));
    faces_kp(face_nearest_kp, faces, nearest_kp, F);
    // for (int i = 0; i < F; i++)
    // {
    //     printf("The shortest keypoint from face %d is: %d \n", i, face_nearest_kp[i]);
    // }

    clock_gettime(CLOCK_REALTIME, &time_stop);
    elapse_time = interval(time_start, time_stop);
    printf("\nTime for Find closest Kp for all Faces: %f(msec)\n", (float)(elapse_time));

    // Output the updated obj file
    output_obj(verts, face_nearest_kp, faces, Ve, F);

    // Clean-up
    free(verts);
    free(faces);
    free(graph);
    free(keypoints);
    free(kv_distances);
    free(keypoint_types);
    free(vk_shortest_dist);
    free(nearest_kp);
    free(face_nearest_kp);

    return 0;
}