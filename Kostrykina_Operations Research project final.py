#!/usr/bin/env python
# coding: utf-8

# In[93]:


from collections import defaultdict


# In[94]:


class Graph:
    def __init__(self, num_nodes):
        self.num_nodes = num_nodes
        self.adj_list = defaultdict(dict)
        self.flow_values = defaultdict(int)

    def add_edge(self, u, v, capacity, cost):
        self.adj_list[u][v] = {'capacity': capacity, 'cost': cost}
        self.adj_list[v][u] = {'capacity': 0, 'cost': -cost}  # Residual edge

    def dijkstra(self, source, sink):
        distance = [float('inf')] * self.num_nodes
        distance[source] = 0
        prev = [-1] * self.num_nodes
        visited = [False] * self.num_nodes

        while True:
            min_distance = float('inf')
            u = -1
            for v in range(self.num_nodes):
                if not visited[v] and distance[v] < min_distance:
                    min_distance = distance[v]
                    u = v

            if u == -1:
                break

            visited[u] = True

            for v in self.adj_list[u]:
                edge = self.adj_list[u][v]
                if edge['capacity'] > 0 and distance[u] + edge['cost'] < distance[v]:
                    distance[v] = distance[u] + edge['cost']
                    prev[v] = u

        return distance, prev

    def augment_path(self, source, sink, prev):
        v = sink
        path_capacity = float('inf')
        while v != source:
            u = prev[v]
            edge = self.adj_list[u][v]
            path_capacity = min(path_capacity, edge['capacity'])
            v = u

        v = sink
        while v != source:
            u = prev[v]
            self.flow_values[(u, v)] += path_capacity
            self.flow_values[(v, u)] -= path_capacity
            self.adj_list[u][v]['capacity'] -= path_capacity
            self.adj_list[v][u]['capacity'] += path_capacity
            v = u

    def max_flow(self, source, sink):
        while True:
            distance, prev = self.dijkstra(source, sink)
            if prev[sink] == -1:
                break
            self.augment_path(source, sink, prev)

        max_flow_value = sum(self.flow_values[(source, v)] for v in self.adj_list[source])
        return max_flow_value

    def find_min_cut(self, source):
        visited = [False] * self.num_nodes
        queue = [source]
        visited[source] = True

        while queue:
            u = queue.pop(0)
            for v in self.adj_list[u]:
                if self.adj_list[u][v]['capacity'] > 0 and not visited[v]:
                    visited[v] = True
                    queue.append(v)

        min_cut = []
        for u in range(self.num_nodes):
            for v in self.adj_list[u]:
                if visited[u] and not visited[v]:
                    min_cut.append((u, v))

        return min_cut

    def min_cost_max_flow(self, source, sink):
        max_flow_value = self.max_flow(source, sink)
        min_cost_value = sum(self.flow_values[(u, v)] * self.adj_list[u][v]['cost']
                             for u in self.adj_list for v in self.adj_list[u])

        return max_flow_value, min_cost_value


# In[95]:


def read_graph_from_file(file_path):
    with open(file_path, 'r') as file:
        num_nodes, num_arcs, source_node, sink_node = map(int, file.readline().split())
        graph = Graph(num_nodes)

        for _ in range(num_arcs):
            u, v, capacity, cost = map(int, file.readline().split())
            graph.add_edge(u, v, capacity, cost)

        # Add the (sink, source) arc if not already present
        if sink_node not in graph.adj_list[source_node]:
            graph.add_edge(sink_node, source_node, 0, 0)

    return num_nodes, source_node, sink_node, graph


# ## Read graph from file

# Change path to the file with a graph.

file_path = 'graph.txt'
num_nodes, source_node, sink_node, graph = read_graph_from_file(file_path)


# ## Maximum Flow
print('1/ Maximum Flow')
print('Implementation of maximum flow algorithm for a graph given by a text file. The result is the max flow value and the list of flow values traversing each arc.')
max_flow_value = graph.max_flow(source_node, sink_node)
flow_values = graph.flow_values
print("Maximum Flow:", max_flow_value)
print("Flow Values Traversing Each Arc:", flow_values)
print()

# ## Minimum Cut
print('2/ Minimum Cut')
print('Implementation of algorithm computing a minimum cut for a graph given by a text file. The result is the list of arcs that form a minimum cut.')
min_cut = graph.find_min_cut(source_node)
print("Minimum Cut:", min_cut)
print()

# ## Maximum Flow Minimum Cost
print('3/ Maximum Flow Minimum Cost')
print('Implementation of maximum flow minimum cost algorithm for a graph given by a text file. The result is the max flow value, the minimum cost value, and the list of flow values traversing each arc.')
max_flow_value, min_cost_value = graph.min_cost_max_flow(source_node, sink_node)
print("Maximum Flow:", max_flow_value)
print("Minimum Cost:", min_cost_value)
print("Flow Values Traversing Each Arc:", flow_values)
