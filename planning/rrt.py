import numpy as np

class Node(object):
  def __init__(self, x, y, parent):
    self.x = x
    self.y = y
    self.parent = parent

class RRT_Tree(object):
  def __init__(self, env, x_start, y_start, radius):
    self.env = env
    self.nodes = []
    self.Root = Node(x_start, y_start, None)
    self.radius = radius
    self.nodes.append(self.Root)

  def add_node(self, x, y, parent):
    self.nodes.append(Node(x, y, parent))

  def find_nearest(self, x, y):
    min_dist = np.inf
    for node in self.nodes:
      dist = np.sqrt((x-node.x)**2 + (y-node.y)**2)
      if dist < min_dist:
        min_dist = dist
        min_node = node
    return min_node

  def match_node(self, node, x, y):
    x_offset = x - node.x
    y_offset = y - node.y
    dist = np.sqrt((x_offset)**2 + (y_offset)**2)
    if dist > self.radius:
      x = node.x + self.radius * x_offset / dist
      y = node.y + self.radius * y_offset / dist
    
    self.add_node(x, y, node)

    if self.env.pass_obstacle(node.x, node.y, x, y) or self.env.check_collision(x, y):
      self.nodes.pop()
      return False
    return True
    

  def scan_node(self, other_node, distance):
    for node in self.nodes:
      if (
        np.sqrt((node.x - other_node.x)**2 + (node.y - other_node.y)**2) < distance and
        not self.env.pass_obstacle(node.x, node.y, other_node.x, other_node.y)
      ):
        return node
    return None
