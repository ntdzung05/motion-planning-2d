import queue
import numpy as np

class Graph(object):
  def __init__(self, env, graph_size, radius):
    self.env = env
    self.graph_size = graph_size
    self.G = []
    self.adjList = []
    self.radius = radius

    self.budget = 1000

    while len(self.G) < self.graph_size and self.budget > 0:
      x = np.random.rand()*self.env.size_x
      y = np.random.rand()*self.env.size_y
      if not self.env.check_collision(x, y):
        self.G.append((x,y))
        self.adjList.append([])
      self.budget -= 1

    self.graph_size = len(self.G)

  def build_graph(self): #undirected
    for startNode in range(self.graph_size):
      for endNode in range(startNode + 1, self.graph_size):
        Collision = False
        euclid_dist = np.sqrt((self.G[startNode][0] - self.G[endNode][0])**2 + (self.G[startNode][1] - self.G[endNode][1])**2)
        if euclid_dist > self.radius:
          continue
        if self.env.pass_obstacle(self.G[startNode][0], self.G[startNode][1], self.G[endNode][0], self.G[endNode][1]):
          Collision = True
        if not Collision:
          self.adjList[startNode].append(endNode)
          self.adjList[endNode].append(startNode)

  def add_node(self, x, y):
    self.G.append((x,y))
    self.adjList.append([])
    self.graph_size += 1
    for node in range(self.graph_size - 1):
      euclid_dist = np.sqrt((self.G[node][0] - x)**2 + (self.G[node][1] - y)**2)
      if euclid_dist > self.radius or self.env.pass_obstacle(self.G[node][0], self.G[node][1], x, y):
        continue
      else:
        self.adjList[node].append(self.graph_size - 1)
        self.adjList[self.graph_size - 1].append(node)

  def remove_last_node(self):
    if self.graph_size == 0:
      return
    node = len(self.G) - 1
    for othernode in self.adjList[node]:
      self.adjList[othernode].remove(node)
    self.adjList.pop(node)
    self.G.pop(node)
    self.graph_size -= 1
  

  def bfs(self, start, goal):
    start = self.G.index(start)
    goal = self.G.index(goal)

    visited = {start: 0}
    path = [[] for _ in range(self.graph_size)]
    path[start] = [start]

    q = queue.Queue()
    q.put(start)
    

    while not q.empty():
      current = q.get()
        
      if current == goal:
        break
      
      for neighbor_node in self.adjList[current]:
        if neighbor_node not in visited or visited[neighbor_node] > visited[current] + 1:
          visited[neighbor_node] = visited[current] + 1
          path[neighbor_node] = path[current].copy()
          path[neighbor_node].append(neighbor_node)
          q.put(neighbor_node)
    return [self.G[node] for node in path[goal]]

  
        
