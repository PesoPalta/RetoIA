#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <limits.h>

#define MAXNODES 4000000
#define MAXNEIGH 45
#define LARGE INT_MAX

typedef struct gnode {
    unsigned id;
    unsigned h1;  // Heurística 1
    unsigned h2;  // Heurística 2
    unsigned gmin; // Mínimo costo acumulado
    unsigned long heapindex;
} gnode;

typedef struct edge {
    unsigned dest;
    unsigned cost1; // Costo 1
    unsigned cost2; // Costo 2
} edge;

// Variables globales
gnode graph[MAXNODES];                  // Nodos del grafo
edge adj[MAXNODES][MAXNEIGH];           // Lista de adyacencias
unsigned degree[MAXNODES] = {0};        // Grado de cada nodo
unsigned num_gnodes = 0;                // Número total de nodos

unsigned stat_expansions = 0;           // Nodos expandidos
unsigned stat_generated = 0;            // Nodos generados
unsigned total_solutions = 0;           // Total soluciones encontradas

// Heap binario para BOA*
typedef struct snode {
    unsigned state; // Nodo actual
    unsigned g1;    // Costo acumulado 1
    unsigned g2;    // Costo acumulado 2
    unsigned key;   // Clave para el heap
} snode;

snode* heap[MAXNODES];
unsigned heapsize = 0;

void insert_heap(snode* n) {
    unsigned i = ++heapsize;
    while (i > 1 && n->key < heap[i / 2]->key) {
        heap[i] = heap[i / 2];
        i /= 2;
    }
    heap[i] = n;
}

snode* remove_min() {
    if (heapsize == 0) return NULL;
    snode* min = heap[1];
    snode* last = heap[heapsize--];
    unsigned i = 1;
    while (2 * i <= heapsize) {
        unsigned child = 2 * i;
        if (child < heapsize && heap[child + 1]->key < heap[child]->key) {
            child++;
        }
        if (last->key <= heap[child]->key) break;
        heap[i] = heap[child];
        i = child;
    }
    heap[i] = last;
    return min;
}

void clear_heap() {
    heapsize = 0;
}

// BOA* para frontera de Pareto
void boa_star(unsigned start, unsigned goal) {
    clear_heap();
    stat_expansions = 0;
    stat_generated = 0;
    total_solutions = 0;

    // Nodo inicial
    snode* start_node = malloc(sizeof(snode));
    start_node->state = start;
    start_node->g1 = 0;
    start_node->g2 = 0;
    start_node->key = 0;
    insert_heap(start_node);

    while (heapsize > 0) {
        snode* current = remove_min();
        unsigned current_id = current->state;

        if (current->g2 >= graph[current_id].gmin) {
            free(current);
            continue;
        }

        graph[current_id].gmin = current->g2;

        if (current_id == goal) {
            total_solutions++;
            free(current);
            continue;
        }

        stat_expansions++;

        for (unsigned i = 0; i < degree[current_id]; i++) {
            edge e = adj[current_id][i];
            unsigned new_g1 = current->g1 + e.cost1;
            unsigned new_g2 = current->g2 + e.cost2;

            if (new_g2 >= graph[e.dest].gmin) continue;

            snode* succ = malloc(sizeof(snode));
            succ->state = e.dest;
            succ->g1 = new_g1;
            succ->g2 = new_g2;
            succ->key = new_g1 + new_g2; // Simple combinación de costos
            insert_heap(succ);
            stat_generated++;
        }

        free(current);
    }
}

// Lectura del archivo de grafo
void read_graph(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Error al abrir el archivo del grafo: %s\n", filename);
        exit(1);
    }

    fscanf(file, "%u", &num_gnodes);
    for (unsigned i = 0; i < num_gnodes; i++) {
        graph[i].id = i;
        graph[i].gmin = LARGE;
    }

    unsigned u, v, c1, c2;
    while (fscanf(file, "%u %u %u %u", &u, &v, &c1, &c2) == 4) {
        adj[u][degree[u]].dest = v;
        adj[u][degree[u]].cost1 = c1;
        adj[u][degree[u]].cost2 = c2;
        degree[u]++;
    }

    fclose(file);
}

// Proceso de una instancia
void process_instance(FILE* output_file, int instance_id, unsigned* nodes, int n) {
    struct timeval tstart, tend;
    gettimeofday(&tstart, NULL);

    stat_expansions = 0;
    stat_generated = 0;
    total_solutions = 0;

    for (int i = 0; i < n; i++) {
        unsigned start = nodes[i];
        unsigned goal = nodes[i + 1];
        boa_star(start, goal);
    }

    gettimeofday(&tend, NULL);
    double exec_time = (tend.tv_sec - tstart.tv_sec) * 1000.0 + (tend.tv_usec - tstart.tv_usec) / 1000.0;

    fprintf(output_file, "%d;%d;%.2f;%u;%u\n",
            instance_id, total_solutions, exec_time, stat_expansions, stat_generated);
}

// Función principal
int main(int argc, char* argv[]) {
    if (argc < 4) {
        printf("Uso: %s <archivo_grafo> <archivo_instancias> <archivo_salida>\n", argv[0]);
        return 1;
    }

    const char* archivo_grafo = argv[1];
    const char* archivo_instancias = argv[2];
    const char* archivo_salida = argv[3];

    read_graph(archivo_grafo);

    FILE* instancias_file = fopen(archivo_instancias, "r");
    FILE* salida_file = fopen(archivo_salida, "w");

    if (!instancias_file || !salida_file) {
        printf("Error al abrir archivo de instancias o salida.\n");
        return 1;
    }

    int instance_id = 1;
    while (!feof(instancias_file)) {
        unsigned nodes[7];
        int n = fscanf(instancias_file, "%u %u %u %u %u %u %u",
                       &nodes[0], &nodes[1], &nodes[2], &nodes[3],
                       &nodes[4], &nodes[5], &nodes[6]);
        if (n >= 3) {
            process_instance(salida_file, instance_id++, nodes, n - 1);
        }
    }

    fclose(instancias_file);
    fclose(salida_file);

    printf("Ejecución completada. Resultados guardados en %s\n", archivo_salida);
    return 0;
}
