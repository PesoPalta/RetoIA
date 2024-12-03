import heapq
import time

# Tamaño máximo de la tabla de nodos (ajustable según necesidades)
MAXNODES = 4000000
MAXNEIGH = 45
MAX_SOLUTIONS = 1000000
LARGE = 1000000000
BASE = 10000000

# Estructura de los nodos del grafo
class GraphNode:
    def __init__(self, id):
        self.id = id
        self.h1 = LARGE
        self.h2 = LARGE
        self.gmin = LARGE
        self.key = LARGE
        self.heapindex = 0  # Usado para el índice en el heap

    def __lt__(self, other):
        """Implementar la comparación entre nodos para que heapq pueda ordenarlos por 'key'."""
        return self.key < other.key

# Estructura de los nodos de la búsqueda BOA*
class SearchNode:
    def __init__(self):
        self.state = None
        self.g1 = 0
        self.g2 = 0
        self.key = 0
        self.heapindex = 0
        self.searchtree = None

    def __lt__(self, other):
        """Implementar la comparación entre nodos para que heapq pueda ordenarlos por 'key'."""
        return self.key < other.key

# Función para manejar la cola de prioridad
class BinaryHeap:
    def __init__(self, max_size):
        self.heap = []
        self.max_size = max_size
        self.size = 0

    def insert(self, node):
        heapq.heappush(self.heap, node)  # Insertamos solo el nodo, sin empaquetarlo en tupla
        node.heapindex = self.size
        self.size += 1

    def pop(self):
        if self.size == 0:
            return None
        node = heapq.heappop(self.heap)  # Extraemos solo el nodo
        self.size -= 1
        return node

    def top(self):
        if self.size == 0:
            return None
        return self.heap[0]

    def empty(self):
        return self.size == 0

# Función para leer el grafo
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
            
            # Restar 1 a los índices para ajustarlos a la indexación de Python (0-indexed)
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

# Función de Dijkstra (para las heurísticas)
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

# Implementación de BOA*
def boastar(adjacent_table, graph_node, start, goal):
    # Inicialización de estructuras
    open_list = BinaryHeap(MAX_SOLUTIONS)
    closed_list = set()
    recycled_nodes = []
    nsolutions = 0
    minf_solution = LARGE
    stat_expansions = 0  # Inicializamos las estadísticas
    stat_generated = 0

    start_node = SearchNode()
    start_node.state = start
    start_node.g1 = 0
    start_node.g2 = 0
    start_node.key = 0
    open_list.insert(start_node)

    while not open_list.empty():
        node = open_list.pop()

        stat_expansions += 1  # Contamos las expansiones de nodos

        if node.g2 >= graph_node[node.state].gmin or node.g2 + graph_node[node.state].h2 >= minf_solution:
            recycled_nodes.append(node)
            continue

        graph_node[node.state].gmin = node.g2

        if node.state == goal:
            solutions.append((node.g1, node.g2))  # Almacenamos la solución en la lista 'solutions'
            nsolutions += 1
            if minf_solution > node.g2:
                minf_solution = node.g2
            continue

        for neighbor, cost1, cost2 in adjacent_table[node.state]:
            new_g1 = node.g1 + cost1
            new_g2 = node.g2 + cost2
            if new_g2 >= graph_node[neighbor].gmin or new_g2 + graph_node[neighbor].h2 >= minf_solution:
                continue

            succ = SearchNode()
            succ.state = neighbor
            succ.g1 = new_g1
            succ.g2 = new_g2
            succ.key = new_g1 + new_g2  # Computar la clave para la cola de prioridad

            stat_generated += 1  # Contamos los nodos generados

            open_list.insert(succ)

    return nsolutions, stat_expansions, stat_generated

# Función para leer las instancias desde el archivo
def parse_instances(file_path):
    instances = []
    with open(file_path, "r") as file:
        for line in file:
            instances.append(list(map(int, line.strip().split())))
    return instances

# Función para guardar los resultados en un archivo
def save_results(results, output_file):
    with open(output_file, "w") as file:
        file.write("\n".join(results))

def main():
    global goal, start, graph_node, solutions
    graph_file = "NY-road-d.txt"
    instances_file = "instancias.txt"
    output_file = "resultados.txt"
    
    # Leer el grafo y preparar los nodos
    adjacent_table, pred_adjacent_table = read_adjacent_table(graph_file)
    graph_node = [GraphNode(i) for i in range(len(adjacent_table))]

    # Cargar las instancias
    instances = parse_instances(instances_file)

    print("Inicio del proceso...")

    results = []
    solutions = []  # Definir la lista de soluciones

    # Ejecutar BOA* para cada instancia
    for i, instance in enumerate(instances, 1):
        start = instance[0]
        intermediates = instance[1:-1]
        goal = instance[-1]

        # Ejecutar Dijkstra para las heurísticas h1 y h2
        backward_dijkstra(1, graph_node, pred_adjacent_table)
        backward_dijkstra(2, graph_node, pred_adjacent_table)

        # Medir el tiempo de ejecución para cada instancia
        instance_start_time = time.time()
        nsolutions, stat_expansions, stat_generated = boastar(adjacent_table, graph_node, start, goal)
        instance_end_time = time.time()

        execution_time = instance_end_time - instance_start_time

        # Guardar los resultados de la instancia
        results.append(f"{i};{nsolutions};{execution_time:.4f};{stat_expansions};{stat_generated}")

        # Notificar que la instancia fue procesada
        print(f"Instancia {i} terminada en {execution_time:.4f} segundos.")

    # Guardar todos los resultados al final
    save_results(results, output_file)

    print("Proceso completado. Los resultados se guardaron en 'resultados.txt'.")

if __name__ == "__main__":
    main()
