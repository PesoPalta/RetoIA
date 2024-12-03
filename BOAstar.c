#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#define MAXNODES 4000000
#define MAXNEIGH 45
#define MAX_SOLUTIONS 1000000
#define MAX_RECYCLE   100000

#define LARGE  1000000000
#define BASE   10000000

#define max(x,y) ( (x) > (y) ? (x) : (y) )
#define min(x,y) ( (x) < (y) ? (x) : (y) )

//********************************************** Main data structures ******************************************************
struct gnode;
typedef struct gnode gnode;

struct gnode {
    long long int id;
    unsigned h1;
    unsigned h2;
    unsigned long long int key;
    unsigned gmin;
    unsigned long heapindex;
};

struct snode;
typedef struct snode snode;

struct snode {
    int state;
    unsigned g1;
    unsigned g2;
    double key;
    unsigned long heapindex;
    snode *searchtree;
};

gnode* graph_node;
unsigned num_gnodes;
unsigned adjacent_table[MAXNODES][MAXNEIGH];
unsigned pred_adjacent_table[MAXNODES][MAXNEIGH];
unsigned goal, start;
gnode* start_state;
gnode* goal_state;
snode* start_node;

unsigned long long int stat_expansions = 0;
unsigned long long int stat_generated = 0;
unsigned long long int minf_solution = LARGE;

unsigned solutions[MAX_SOLUTIONS][2];
unsigned nsolutions = 0;
unsigned stat_pruned = 0;
unsigned stat_created = 0;

//********************************************** Binary Heap Data Structures ******************************************************

#define HEAPSIZEDIJ 3000000
gnode* heap_dij[HEAPSIZEDIJ];
unsigned long int heapsize_dij = 0;
unsigned long int stat_percolations = 0;

void percolatedown_dij(int hole, gnode* tmp) {
    int child;
    if (heapsize_dij != 0) {
        for (; 2 * hole <= heapsize_dij; hole = child) {
            child = 2 * hole;
            if (child != heapsize_dij && heap_dij[child + 1]->key < heap_dij[child]->key) ++child;
            if (heap_dij[child]->key < tmp->key) {
                heap_dij[hole] = heap_dij[child];
                heap_dij[hole]->heapindex = hole;
                ++stat_percolations;
            } else break;
        }
        heap_dij[hole] = tmp;
        heap_dij[hole]->heapindex = hole;
    }
}

void percolateup_dij(int hole, gnode* tmp) {
    if (heapsize_dij != 0) {
        for (; hole > 1 && tmp->key < heap_dij[hole / 2]->key; hole /= 2) {
            heap_dij[hole] = heap_dij[hole / 2];
            heap_dij[hole]->heapindex = hole;
            ++stat_percolations;
        }
        heap_dij[hole] = tmp;
        heap_dij[hole]->heapindex = hole;
    }
}

void insertheap_dij(gnode* thiscell) {
    if (thiscell->heapindex == 0) percolateup_dij(++heapsize_dij, thiscell);
    else percolatedown_dij(thiscell->heapindex, heap_dij[thiscell->heapindex]);
}

gnode* popheap_dij() {
    gnode* thiscell;
    if (heapsize_dij == 0) return NULL;
    thiscell = heap_dij[1];
    thiscell->heapindex = 0;
    percolatedown_dij(1, heap_dij[heapsize_dij--]);
    return thiscell;
}

//********************************************** BOA* Functions ******************************************************

void read_adjacent_table(const char* filename) {
    FILE* f;
    int i, ori, dest, dist, t;
    f = fopen(filename, "r");
    if (f == NULL) {
        printf("Cannot open file %s.\n", filename);
        exit(1);
    }
    fscanf(f, "%d %d", &num_gnodes, NULL);
    for (i = 0; i < num_gnodes; i++) adjacent_table[i][0] = 0;
    while (fscanf(f, "%d %d %d %d\n", &ori, &dest, &dist, &t) == 4) {
        adjacent_table[ori - 1][0]++;
        adjacent_table[ori - 1][adjacent_table[ori - 1][0] * 3 - 2] = dest - 1;
        adjacent_table[ori - 1][adjacent_table[ori - 1][0] * 3 - 1] = dist;
        adjacent_table[ori - 1][adjacent_table[ori - 1][0] * 3] = t;
    }
    fclose(f);
}

void new_graph() {
    graph_node = (gnode*) calloc(num_gnodes, sizeof(gnode));
    for (int y = 0; y < num_gnodes; ++y) {
        graph_node[y].id = y;
        graph_node[y].gmin = LARGE;
        graph_node[y].h1 = LARGE;
        graph_node[y].h2 = LARGE;
    }
}

void backward_dijkstra(int dim) {
    for (int i = 0; i < num_gnodes; ++i) graph_node[i].key = LARGE;
    emptyheap_dij();
    goal_state->key = 0;
    insertheap_dij(goal_state);
    while (topheap_dij() != NULL) {
        gnode* n = popheap_dij();
        if (dim == 1) n->h1 = n->key;
        else n->h2 = n->key;
        for (short d = 1; d < pred_adjacent_table[n->id][0] * 3; d += 3) {
            gnode* pred = &graph_node[pred_adjacent_table[n->id][d]];
            int new_weight = n->key + pred_adjacent_table[n->id][d + dim];
            if (pred->key > new_weight) {
                pred->key = new_weight;
                insertheap_dij(pred);
            }
        }
    }
}

void initialize_parameters() {
    start_state = &graph_node[start];
    goal_state = &graph_node[goal];
}

void call_boastar() {
    initialize_parameters();
    backward_dijkstra(1);
    backward_dijkstra(2);
    boastar();
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Uso: %s <archivo_instancias> <archivo_salida>\n", argv[0]);
        return 1;
    }

    FILE* instancias_file = fopen(argv[1], "r");
    FILE* salida_file = fopen(argv[2], "w");
    if (!instancias_file || !salida_file) {
        printf("Error al abrir archivos\n");
        return 1;
    }

    read_adjacent_table("NY-road-d.txt");
    new_graph();

    int instancia_id = 1;
    while (!feof(instancias_file)) {
        int n, destinos[7];
        if (fscanf(instancias_file, "%d %d %d %d %d %d %d", &destinos[0], &destinos[1], &destinos[2], &destinos[3], &destinos[4], &destinos[5], &destinos[6]) == 7) {
            start = destinos[0];
            goal = destinos[6];

            struct timeval tstart, tend;
            gettimeofday(&tstart, NULL);

            unsigned total_expansions = 0;
            unsigned total_generated = 0;
            nsolutions = 0;

            for (int i = 0; i < 6; i++) {
                start = destinos[i];
                goal = destinos[i + 1];
                call_boastar();
                total_expansions += stat_expansions;
                total_generated += stat_generated;
            }

            gettimeofday(&tend, NULL);
            double exec_time = (tend.tv_sec - tstart.tv_sec) * 1000.0 + (tend.tv_usec - tstart.tv_usec) / 1000.0;

            fprintf(salida_file, "%d;%d;%.2f;%llu;%llu\n", instancia_id++, nsolutions, exec_time, total_expansions, total_generated);
        }
    }

    fclose(instancias_file);
    fclose(salida_file);

    return 0;
}
