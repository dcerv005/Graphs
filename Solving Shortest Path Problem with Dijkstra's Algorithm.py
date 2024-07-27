import heapq
import timeit
class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        if vertex in self.vertices:
            return self.vertices[vertex]
        else:
            return {}
        
    def dijkstra (self, start):
        distances= {vertex:float('inf') for vertex in self.vertices}
        distances[start]=0
        pq = [(0, start)]

        while pq:
            current_distance, current_vertex = heapq.heappop(pq)
            if current_distance > distances[current_vertex]:
                continue
            for neighbor, weight in self.vertices[current_vertex].items():
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor]=distance
                    heapq.heappush(pq, (distance, neighbor))
            return distances

# Example usage:
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_edge('A', 'B', 5)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)

print(graph.vertices)  # Output: {'A': {'B': 5, 'C': 10}, 'B': {'A': 5, 'C': 3}, 'C': {'B': 3, 'A': 10}}

# distances= graph.dijkstra('A')
time=timeit.timeit(lambda: print("Distances from vertex'B'", graph.dijkstra('B')), number=1)
# print("Distancese from vertex 'A':", distances) 

print(time)

#Space complexity: O(n), time complexity O(nlogn)