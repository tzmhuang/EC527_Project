// gcc -O1 obj_to_adjlist.c -lrt -std=c99 -o obj_to_adjlist

// Adjacency List code borrowed from https://www.geeksforgeeks.org/graph-and-its-representations/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define INT_MAX 1e+6

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
// the AdjList only consist of the head
struct AdjList
{
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
        graph->array[i].head = NULL;
    }

    return graph;
}

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
        }
    }

    // Since graph is undirected, add an edge from dest to src also
    newNode = newAdjListNode(src, dist);
    dupe = 0;
    if (graph->array[dest].head == NULL)
    {
        graph->array[dest].head = newNode;
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

void PrintDistance(float dist[], int n)
{
	// This function prints the final solution
    printf("The result of the BellmanFord and a given vert is: ");
    printf("\nVertex\tDistance from Source Vertex\n");
    int i;
 
    for (i = 0; i < n; ++i){
		printf("%d \t\t %f\n", i+1, dist[i]);
	}
}


void BellmanFord(struct Graph* graph, int index)
{
    int V = graph->Ve;
    int to;
    struct AdjListNode *from = NULL;
    data_t weight;
    float StoreDistance[V];
    int i,j;
    int source = index - 1;
 
    // This is initial step that we know , we initialize all distance to infinity except source.
	// We assign source distance as 0(zero)
 
    for (i = 0; i < V; i++)
        StoreDistance[i] = INT_MAX;
 
    StoreDistance[source] = 0;

 
    from = graph->array[source].head;

    while(from != NULL){
        to = from->dest;
        weight = from->dist;
        if(StoreDistance[to] > StoreDistance[source] + weight){
            StoreDistance[to] = StoreDistance[source] + weight;
        }
        from = from->next;
    }


    for(int i = 0; i < V; i++){
        from = graph->array[i].head;
        while(from != NULL){
            to = from->dest;
            weight = from->dist;
            if(StoreDistance[to] > StoreDistance[source] + weight){
                // Update the shortest path distance
                StoreDistance[to] = StoreDistance[source] + weight;
            }
            from = from->next;
        }
    }
 
    PrintDistance(StoreDistance, V);
 
    return;
}



int main()
{
    FILE *fp;
    char *line = NULL;
    size_t len = 0;
    size_t read;
    int Ve = 0, F = 0, j = 0, k = 0, ret;
    char discard;

    fp = fopen("pyramid.obj", "r");
    // fp = fopen("child_cam_frame.obj", "r");
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

    int *matrix_edge = (int *)malloc(Ve * Ve * sizeof(int));           

    struct Graph *graph = createGraph(Ve);

    // Close the file and clean-up a little
    fclose(fp);
    if (line)
        free(line);

    // Print array of vertices
    printf("\nVertices:\n");
    j = 0;
    for (int i = 0; i < Ve; i++)
    {
        printf("%d: %f, %f, %f\n", i, verts[j], verts[j + 1], verts[j + 2]);
        j += 3;
    }

    // Print array of faces
    printf("\nFaces:\n");
    k = 0;
    for (int i = 0; i < F; i++)
    {
        printf("%d: %d, %d, %d\n", i, faces[k], faces[k + 1], faces[k + 2]);
        k += 3;
    }



    // Add edges to adjacency matrix
    k = 0;
    for (int i = 0; i < F; i++)
    {
        addEdges(verts, graph, faces[k], faces[k + 1], faces[k + 2]);
        k += 3;
    }

    // Print adjacency list
    printGraph(graph);
    printf("\n");

    BellmanFord(graph, 1);

    // Clean-up
    free(verts);
    free(faces);
    free(graph);

    return 0;
}