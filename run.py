import heapq
import time

MAXNODES = 4000000
MAXNEIGH = 45
MAX_SOLUTIONS = 1000000
LARGE = 1000000000
BASE = 10000000

class GraphNode:
    def __init__(self, id):
        self.id = id
        self.h1 = LARGE
        self.h2 = LARGE
        self.gmin = LARGE
        self.key = LARGE
        self.heapindex = 0

    def __lt__(self, other):
        return self.key < other.key

class SearchNode:
    def __init__(self):
        self.state = None
        self.g1 = 0
        self.g2 = 0
        self.key = 0
        self.heapindex = 0
        self.searchtree = None

    def __lt__(self, other):
        return self.key < other.key

class BinaryHeap:
    def __init__(self, max_size):
        self.heap = []
        self.max_size = max_size
        self.size = 0

    def insert(self, node):
        heapq.heappush(self.heap, node)
        node.heapindex = self.size
        self.size += 1

    def pop(self):
        if self.size == 0:
            return None
        node = heapq.heappop(self.heap)
        self.size -= 1
        return node

    def top(self):
        if self.size == 0:
            return None
        return self.heap[0]

    def empty(self):
        return self.size == 0

def read_adjacent_table(filename):
    global num_gnodes
    adjacent_table = {}
    pred_adjacent_table = {}

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            ori, dest, dist, t = map(int, line.split())
            ori -= 1
            dest -= 1

            if ori not in adjacent_table:
                adjacent_table[ori] = []
                pred_adjacent_table[ori] = []
            if dest not in adjacent_table:
                adjacent_table[dest] = []
                pred_adjacent_table[dest] = []

            adjacent_table[ori].append((dest, dist, t))
            pred_adjacent_table[dest].append((ori, dist, t))
    
    num_gnodes = len(adjacent_table)
    return adjacent_table, pred_adjacent_table

def backward_dijkstra(dim, graph_node, pred_adjacent_table):
    for node in graph_node:
        node.key = LARGE
    heap = BinaryHeap(len(graph_node))
    graph_node[goal].key = 0
    heap.insert(graph_node[goal])

    while not heap.empty():
        node = heap.pop()
        if dim == 1:
            node.h1 = node.key
        else:
            node.h2 = node.key
        for neighbor, dist, time in pred_adjacent_table[node.id]:
            neighbor_node = graph_node[neighbor]
            new_weight = node.key + (dist if dim == 1 else time)
            if neighbor_node.key > new_weight:
                neighbor_node.key = new_weight
                heap.insert(neighbor_node)

def boastar(adjacent_table, graph_node, start, goal, solutions):
    open_list = BinaryHeap(MAX_SOLUTIONS)
    recycled_nodes = []
    nsolutions = 0
    minf_solution = LARGE
    stat_expansions = 0
    stat_generated = 0

    start_node = SearchNode()
    start_node.state = start
    start_node.g1 = 0
    start_node.g2 = 0
    start_node.key = 0
    open_list.insert(start_node)

    # Reiniciar gmin en todos los nodos
    for node in graph_node:
        node.gmin = LARGE

    while not open_list.empty():
        node = open_list.pop()
        stat_expansions += 1

        if node.g2 >= graph_node[node.state].gmin:
            recycled_nodes.append(node)
            continue

        graph_node[node.state].gmin = node.g2

        if node.state == goal:
            solutions.append((node.g1, node.g2))
            nsolutions += 1
            if minf_solution > node.g2:
                minf_solution = node.g2
            continue

        for neighbor, cost1, cost2 in adjacent_table[node.state]:
            new_g1 = node.g1 + cost1
            new_g2 = node.g2 + cost2

            if new_g2 >= graph_node[neighbor].gmin:
                continue

            succ = SearchNode()
            succ.state = neighbor
            succ.g1 = new_g1
            succ.g2 = new_g2
            succ.key = new_g1 + graph_node[neighbor].h1 + new_g2 + graph_node[neighbor].h2

            stat_generated += 1
            open_list.insert(succ)

    return nsolutions, stat_expansions, stat_generated

def parse_instances(file_path):
    instances = []
    with open(file_path, "r") as file:
        for line in file:
            instance = list(map(int, line.strip().split()))
            if len(instance) >= 2:
                instances.append(instance)
    return instances

def save_results(results, output_file):
    with open(output_file, "w") as file:
        file.write("\n".join(results))

def process_queries(query_files):
    global goal, start, graph_node
    graph_file = "NY-road-d.txt"
    
    adjacent_table, pred_adjacent_table = read_adjacent_table(graph_file)
    graph_node = [GraphNode(i) for i in range(len(adjacent_table))]

    for query_file in query_files:
        instances = parse_instances(query_file["input_file"])
        output_file = query_file["output_file"]

        print(f"Procesando {query_file['input_file']}...")

        results = []

        for i, instance in enumerate(instances, 1):
            if len(instance) < 2:
                print(f"Instancia {i} está incompleta. Omitida.")
                continue

            total_expansions = 0
            total_generated = 0
            total_time = 0.0
            nsolutions = 0

            start = instance[0]
            intermediates = instance[1:-1]
            goal = instance[-1]
            current_start = start

            for current_goal in intermediates + [goal]:
                solutions = []

                # Ejecutar backward Dijkstra para heurísticas h1 y h2
                backward_dijkstra(1, graph_node, pred_adjacent_table)
                backward_dijkstra(2, graph_node, pred_adjacent_table)

                # Ejecutar BOA*
                sub_start_time = time.time()
                nsolutions_sub, stat_expansions, stat_generated = boastar(
                    adjacent_table, graph_node, current_start, current_goal, solutions
                )
                sub_end_time = time.time()

                # Actualizar métricas
                total_time += sub_end_time - sub_start_time
                total_expansions += stat_expansions
                total_generated += stat_generated
                nsolutions += nsolutions_sub

                print(f"Procesado desde {current_start} hasta {current_goal} en {sub_end_time - sub_start_time:.4f} segundos.")

                # Actualizar el inicio para el siguiente subproblema
                current_start = current_goal

            results.append(f"{i};{nsolutions};{total_time:.4f};{total_expansions};{total_generated}")
            print(f"Instancia {i} terminada en {total_time:.4f} segundos.")

        save_results(results, output_file)

        print(f"Proceso completado para {query_file['input_file']}. Los resultados se guardaron en '{output_file}'.")

def main():
    query_files = [
        {"input_file": "Queries\\NY-queries-1p.txt", "output_file": "Resultados\\resultados-query1.txt"}, # EN ESTE LISTADO SE PUEDE AGREGAR O QUITAR CUALQUIERA DE LOS ARCHIVOS
        {"input_file": "Queries\\NY-queries-2p.txt", "output_file": "Resultados\\resultados-query2.txt"},
        {"input_file": "Queries\\NY-queries-3p.txt", "output_file": "Resultados\\resultados-query3.txt"}
    ]
    
    process_queries(query_files)

if __name__ == "__main__":
    main()