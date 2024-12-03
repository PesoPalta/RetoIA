#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#define MAXNODES 4000000
#define MAXNEIGH 45
#define LARGE 1000000000

struct gnode {
    long long int id;
    unsigned h1;
    unsigned h2;
    unsigned long long int key;
    unsigned gmin;
    unsigned long heapindex;
};

struct gnode* heap_dij[MAXNODES];
unsigned heapsize_dij = 0;

void emptyheap_dij() {
    heapsize_dij = 0;
}

struct gnode* topheap_dij() {
    if (heapsize_dij == 0)
        return NULL;
    return heap_dij[1];
}

void read_adjacent_table(const char* filename) {
    FILE* f;
    int num_gnodes;
    f = fopen(filename, "r");
    if (!f) {
        printf("Cannot open file %s.\n", filename);
        exit(1);
    }
    if (fscanf(f, "%d", &num_gnodes) != 1) {
        printf("Error al leer el número de nodos.\n");
        exit(1);
    }
    fclose(f);
}

void boastar() {
    // Implementación del algoritmo BOA*.
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Uso: %s <archivo_instancias> <archivo_salida>\n", argv[0]);
        return 1;
    }

    read_adjacent_table("NY-road-d.txt");
    emptyheap_dij();
    topheap_dij();

    boastar();
    return 0;
}

