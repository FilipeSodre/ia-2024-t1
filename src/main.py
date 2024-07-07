import math
import os

# Heurística
def haversine(lat1, lon1, lat2, lon2):
    """Calcula a distância, em metros, entre duas coordenadas GPS."""
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # apply formulae
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    
    r = 6371  
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return r * c * 1000  

# Ler grafo 
def read_graph(filename):
    graph = {}
    coordinates = {}

    with open(filename, 'r') as file:
        lines = file.readlines()
        num_nodes = int(lines[0].strip())
        edge_start = num_nodes + 1
        
        for i in range(1, num_nodes + 1):
            parts = lines[i].strip().split()
            node = int(parts[0])
            lat = float(parts[1])
            lon = float(parts[2])
            coordinates[node] = (lat, lon)
        
        for line in lines[edge_start:]:
            parts = line.strip().split()
            if len(parts) == 3:
                node1, node2, weight = map(float, parts)
                if int(node1) not in graph:
                    graph[int(node1)] = []
                if int(node2) not in graph:
                    graph[int(node2)] = []
                graph[int(node1)].append((int(node2), weight))
                graph[int(node2)].append((int(node1), weight))
    
    return graph, coordinates

# Algoritmo DFS
def dfs(graph, start, goal):
    visited = set()
    stack = [(start, [start], 0)]  

    while stack:
        node, path_so_far, path_weight = stack.pop()

        if node == goal:
            return len(visited), path_weight, path_so_far

        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node], key=lambda x: x[0])  

            for neighbor, weight in neighbors:
                if neighbor not in visited:
                    stack.append((neighbor, path_so_far + [neighbor], path_weight + weight))

    return len(visited), float('inf'), []

# Algoritmo BFS
def bfs(graph, start, goal):
    visited = list()
    queue = [(start, [start], 0)]  

    while queue:
        node, path_so_far, path_weight = queue.pop(0)

        if node == goal:
            return len(visited), path_weight, path_so_far

        if node not in visited:
            visited.append(node)
            neighbors = sorted(graph[node], key=lambda x: x[0])  

            for neighbor, weight in neighbors:
                if neighbor not in visited:
                    queue.append((neighbor, path_so_far + [neighbor], path_weight + weight))

    return len(visited), float('inf'), []


# Algoritmo Dijkstra
def dijkstra(graph, start: int, goal: int):
    visited = set()
    dist = {v: float('inf') for v in graph}
    prev = {v: None for v in graph}
    dist[start] = 0
    queue = [(0, start)]  
    
    while queue:
        
        min_dist = float('inf')
        min_node = None
        for dist_u, u in queue:
            if dist_u < min_dist:
                min_dist = dist_u
                min_node = u
        
        u = min_node
        queue = [(dist_u, node) for dist_u, node in queue if node != u]
        
        if u == goal:
            break
        if u in visited:
            continue
        visited.add(u)
        
        for neighbor, weight in graph[u]:
            if neighbor not in visited:
                alt = dist[u] + weight
                if alt < dist[neighbor]:
                    dist[neighbor] = alt
                    prev[neighbor] = u
                    queue.append((alt, neighbor))
    
    path = []
    u = goal
    if prev[u] or u == start:
        while u is not None:
            path.insert(0, u)
            u = prev[u]
        if start not in path:
            path.insert(0, start)

    num_visited = len(visited)
    path_weight = dist[goal] if dist[goal] != float('inf') else None

    return num_visited, path_weight, path

# Algoritmo A* 
def a_star(graph, coordinates, start, goal):
    open_set = [(0, start, [start], 0)] 
    came_from = {}
    g_score = {start: 0}
    f_score = {start: haversine(coordinates[start][0], coordinates[start][1], coordinates[goal][0], coordinates[goal][1])}

    while open_set:
        open_set.sort()  
        _, current, path_so_far, path_weight = open_set.pop(0)

        if current == goal:
            return len(path_so_far), path_weight, path_so_far

        if current in graph:
            for neighbor, weight in graph[current]:
                tentative_g_score = g_score.get(current, float('inf')) + weight
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    h_score = haversine(coordinates[neighbor][0], coordinates[neighbor][1], coordinates[goal][0], coordinates[goal][1])
                    f_score[neighbor] = tentative_g_score + h_score
                    open_set.append((f_score[neighbor], neighbor, path_so_far + [neighbor], path_weight + weight))

    return float('inf'), float('inf'), []


if __name__ == "__main__":
    filename = os.path.join(os.path.dirname(__file__), '..', 'mapas', 'small_map.txt')
    graph, coordinates = read_graph(filename)
    start_node = 0
    goal_node = 20

    # DFS
    try:
        nodes_visited, path_length, path_taken = dfs(graph, start_node, goal_node)
        print(f"\nNós visitados (DFS): {nodes_visited}")
        print(f"Comprimento do caminho (DFS): {path_length}")
        print(f"Caminho encontrado (DFS): {path_taken}")
    except ValueError as e:
        print(e)

    # BFS
    try:
        nodes_visited, path_length, path_taken = bfs(graph, start_node, goal_node)
        print(f"\nNós visitados (BFS): {nodes_visited}")
        print(f"Comprimento do caminho (BFS): {path_length}")
        print(f"Caminho encontrado (BFS): {path_taken}")
    except ValueError as e:
        print(e)

    # Dujkstra
    try:
        nodes_visited, path_length, path_taken = dijkstra(graph, start_node, goal_node)
        print(f"\nNós visitados (Dijkstra): {nodes_visited}")
        print(f"Comprimento do caminho (Dijkstra): {path_length}")
        print(f"Caminho encontrado (Dijkstra): {path_taken}")
    except ValueError as e:
        print(e)

    # A*
    try:
        nodes_visited, path_length, path_taken = a_star(graph, coordinates, start_node, goal_node)
        print(f"\nNós visitados (A*): {nodes_visited}")
        print(f"Comprimento do caminho (A*): {path_length}")
        print(f"Caminho encontrado (A*): {path_taken}")
    except ValueError as e:
        print(e)
