#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
// #include <omp.h>


#define DIM 3

typedef float data_t;

typedef struct {
  long int len;
  data_t *data;
} array_rec, *array_ptr;


void load_points(char* file_name, array_ptr* point_array);
void get_distance(array_ptr kp_array, array_ptr gp_array, array_ptr* res);

int main(int argc, char *argv[]){
    array_ptr kp_array; // N_kp by DIM array
    array_ptr gp_array; // N_gp by DIM array
    array_ptr d_array; // N_kp by N_gp array

    printf("load kp array: %s\n", argv[1]);
    load_points(argv[1], &kp_array);

    printf("load gp array: %s\n", argv[2]);
    load_points(argv[2], &gp_array);


    for (int i=0; i< kp_array->len; i++){
        for (int j=0; j<DIM; j++){
            printf("%f ", kp_array->data[i*DIM + j]);
        }
        printf("\n");
    }


    for (int i=0; i<gp_array->len; i++){
        for (int j=0; j<DIM; j++){
            printf("%f ", gp_array->data[i*DIM + j]);
        }
        printf("\n");
    }

    get_distance(kp_array, gp_array, &d_array);

    printf("\n");
    printf("\n");
    printf("%ld \n",kp_array->len);
    printf("%ld \n",gp_array->len);
    printf("%ld \n",d_array->len);

    for(int i=0; i<kp_array->len; i++){
        for(int j=0; j<gp_array->len; j++){
            printf("%f ",d_array->data[i*(gp_array->len) + j]);
        }
        printf("\n");
    }

    return 0;
}

void get_distance(array_ptr kp_array, array_ptr gp_array, array_ptr* res){
    int N_kp = kp_array->len;
    int N_gp = gp_array->len;
    data_t d, kp_x, kp_y, kp_z, gp_x, gp_y, gp_z;
    data_t* data = (data_t *) calloc(N_kp*N_gp, sizeof(data_t));
    array_ptr array = (array_ptr) malloc(sizeof(array_rec));
    array->len = N_kp;
    array->data = data;

    for (int i=0; i<N_kp; i++){
        kp_x = kp_array->data[i*DIM];
        kp_y = kp_array->data[i*DIM+1];
        kp_z = kp_array->data[i*DIM+2];
        
        for (int j=0; j<N_gp; j++){
            gp_x = gp_array->data[j*DIM];
            gp_y = gp_array->data[j*DIM+1];
            gp_z = gp_array->data[j*DIM+2];
            array->data[i*N_gp + j] = (kp_x-gp_x)*(kp_x-gp_x) + (kp_y-gp_y)*(kp_y-gp_y) + (kp_z-gp_z)*(kp_z-gp_z);
        }
    }

    *res = array;
}

void load_points(char* file_name, array_ptr* point_array){
    FILE* fptr;
    int N;
    data_t n;
    fptr = fopen(file_name, "r");

    if (NULL == fptr){
        printf("file cannot be opened \n");
    }
    if (!fscanf(fptr, "%d ", &N)){ printf("Error!"); return; }
    printf("File contain %d points\n", N);
    data_t* data = (data_t *) calloc(N*DIM, sizeof(data_t));
    array_ptr array = (array_ptr) malloc(sizeof(array_rec));
    array->len = N;
    array->data = data;
    

    for (int i=0; i<N; i++){
        for (int j=0; j<DIM; j++){
            if(!fscanf(fptr, "%f ", &n)){ printf("Error!"); return;};
            // printf("%f ", n);
            array->data[i*DIM + j] = n;
        }
        // printf("\n");
    }

    // printf("\n");
    fclose(fptr);

    *point_array = array;
}