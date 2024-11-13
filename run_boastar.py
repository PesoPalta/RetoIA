import subprocess

def run_boastar(start, goal, graph_file="NY-road-d.txt"):
    # Ejecuta el programa BOAstar con los argumentos y captura la salida
    result = subprocess.run(["./BOAstar", str(start), str(goal)], capture_output=True, text=True)
    if result.returncode == 0:
        output = result.stdout
        print("Resultados de BOA*:", output)
    else:
        print("Error en la ejecución de BOA*:", result.stderr)

# Ejemplo de uso
if __name__ == "__main__":
    start = 180833
    goal = 83149
    run_boastar(start, goal)
