"""Implementação de uma estrutura de grafo."""

import sys


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

