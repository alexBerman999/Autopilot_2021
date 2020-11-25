import math
import random
import matplotlib.pyplot as plt
from collections import defaultdict
from collections import deque

near_vertex = []
obstacle_list = []


class EuclideanEdge:
    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.weight = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def get_weight(self):
        return self.weight

    def get_to(self):
        return self.x2, self.y2

    def get_from(self):
        return self.x1, self.y1


class EuclideanEdgeWeightedGraph:

    def __init__(self, v, e):
        self.v = v
        self.e = e
        self.adj = defaultdict(list)
        self.weights = defaultdict(float)
        self.vertices = []
        self.marked = defaultdict(bool)
        self.parents = {}
        self.path_costs = {}

    def get_v(self):
        return self.v

    def get_e(self):
        return self.e

    def get_vertex(self, x, y):
        return self.vertices.index((x,y))

    def add_edge(self, edge):
        v = edge.get_to()
        w = edge.get_from()
        if v not in self.vertices:
            self.v += 1
            self.vertices.append(v)
        if w not in self.vertices:
            self.v += 1
            self.vertices.append(w)
        self.adj[self.vertices.index(v)].append(self.vertices.index(w))
        self.parents[v] = w
        self.weights[self.v] = edge.weight
    
    def add_edge_xy(self, x,y,x2,y2):
        to= (x,y)
        fromnode = (x2,y2)
        if to not in self.vertices:
             self.v += 1 
             self.vertices.append(to)
        if fromnode not in self.vertices: 
             self.v += 1 
             self.vertices.append(fromnode)
        v = self.vertices.index(to)
        w = self.vertices.index(fromnode)
        self.adj[v].append(w)
        self.parents[fromnode] = to 
        self.weights[self.vertices.index(fromnode)] = math.hypot((x2-x), (y2-y))
        self.path_costs[(x2,y2)] = math.hypot((x2-x), (y2-y)) + self.path_costs[(x,y)]
    
    def add_init(self, x, y):
        self.v += 1
        self.vertices.append((x,y))
        self.parents[(0,0)] = None
        self.path_costs[(0,0)] = 0 
    
    def update_cost(self, v):
        elem= self.vertices[v]
        queue = deque([])
        cost = self.weights[v]
        queue.appendleft(self.parents[elem])
        while queue and self.parents[elem] is not None:
            elem = queue.pop()
            vertex = self.get_vertex(elem[0], elem[1])
            cost += self.weights[vertex]
            queue.appendleft(self.parents[(elem[0], elem[1])])
        return cost
    
    def update_cost_xy(self, x, y): 
        for key,value in self.parents.items(): 
            if value == (x,y): 
                child_node = key
                self.path_costs[child_node] = math.hypot(child_node[0]-x, child_node[1]-y) + self.path_costs[(x,y)]

class RRTStar:

    def __init__(self, w, x, y, z, n, r, obstacle_list):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        self.n = n
        self.r = r
        self.rrt_graph = EuclideanEdgeWeightedGraph(0, 0)
        self.obstacle_list = obstacle_list

    def planning(self):
        self.rrt_graph.add_init(self.w, self.y)
        for i in range(self.n):
            x_new = random.uniform(0,40)
            y_new = random.uniform(0,40)
            if self.collision_detection( x_new, y_new):
                node, cost_list = self.nearest(x_new, y_new)
                if self.collision_detection_eq(node[0], node[1], x_new, y_new):
                    self.steer(node[0], node[1], x_new, y_new)
                    near_nodes = self.find_near_nodes(cost_list)
                    self.update_parent(x_new,y_new,near_nodes)
                    self.rewire(x_new, y_new, near_nodes)
                    if self.calc_cost(self,x_new, y_new, self.x, self.z) < self.r and i > 200: 
                        self.search_best_goal_node()                  
                        break 
        if (self.x, self.z) not in list(self.rrt_graph.parents.keys()):
            self.search_best_goal_node()
        final_tree = self.reconstruct_path()
        return final_tree,self.rrt_graph.path_costs[(self.x, self.z)]
    
    def update_parent(self,x, y,near_nodes):
        costs = []
        for elem in near_nodes: 
            edge_cost = self.calc_cost(self,elem[0], elem[1], x,y)
            cost = self.rrt_graph.path_costs[elem] + edge_cost 
            if self.collision_detection_eq(elem[0], elem[1], x,y) and (elem[0], elem[1]) != (x,y): 
                costs.append(cost)
            else: 
                costs.append(100000)
        if costs:
            min_cost = min(costs)
            if min_cost != 100000:
                min_ind = costs.index(min_cost)
                self.steer(near_nodes[min_ind][0], near_nodes[min_ind][1], x,y)
    
    def search_best_goal_node(self): 
        goal_near_nodes = []
        for elem in self.rrt_graph.vertices: 
            if (self.calc_cost(self, elem[0], elem[1], self.x, self.z) < (self.r*3)):
                if self.collision_detection_eq(elem[0], elem[1], self.x, self.z): 
                    goal_near_nodes.append(elem)
        self.update_parent(self.x,self.z, goal_near_nodes)


    def find_near_nodes(self,cost_list): 
        nearest_vertices = []
        for i in range(len(cost_list)):
            if cost_list[i] < (self.r*2):
                nearest_vertices.append(self.rrt_graph.vertices[i])
        return nearest_vertices
    
    def nearest(self, x, y):
        point_costs = defaultdict(float)
        length = len(self.rrt_graph.vertices)
        for i in range(length):
            point_costs[i] = self.calc_cost(self, self.rrt_graph.vertices[i][0], self.rrt_graph.vertices[i][1], x, y)
        min_cost = min(list(point_costs.values()))
        graph_node = 0 
        for key, value in point_costs.items():
            if value == min_cost: 
                graph_node = key
                break 
        graph_node = self.rrt_graph.vertices[graph_node]
        return graph_node, point_costs

    def steer(self, x,y,x2,y2): 
        if self.calc_cost(self,x,y,x2,y2) > self.r: 
            theta = math.atan((y2-y)/(x2-x))
            partition = self.calc_cost(self,x,y,x2,y2)/self.r
            cachex = 0
            cachey = 0
            for i in range(int(partition)): 
                x1 = x + (i*self.r * math.cos(theta))
                y1 = y+ (i*self.r * math.sin(theta))    
                x12 = x + ((i+1)*self.r * math.cos(theta))
                y12 = y + ((i+1)*self.r * math.sin(theta))
                self.rrt_graph.add_edge_xy(x1, y1, x12, y12)  
                cachex = x12 
                cachey = y12
            self.rrt_graph.add_edge_xy(cachex, cachey, x2, y2)    
        else: 
            self.rrt_graph.add_edge_xy(x,y, x2,y2)
    
    @staticmethod
    def calc_cost(self, x, y, x2, y2):
        cost = math.hypot(abs((x2 - x)),abs((y2-y)))
        return cost

    def rewire(self, x, y, near_vertices):
        for elem in near_vertices:
            initial_cost = self.rrt_graph.path_costs[elem]
            upto_cost = self.rrt_graph.path_costs[(x,y)]
            cost = self.calc_cost(self,x, y, elem[0], elem[1]) + upto_cost
            if (cost<initial_cost) and self.collision_detection_eq(x, y, elem[0], elem[1]):
                self.steer(x,y, elem[0], elem[1])
                self.rrt_graph.update_cost_xy(elem[0], elem[1])
    
    def collision_detection(self, x, y):
        for elem in self.obstacle_list:
            value = (x - elem[2])**2 + (y-elem[3])**2
            if value <= (elem[1] ** 2): 
                return False
            else:
                return True

    def collision_detection_eq(self,x1, y1, x2, y2):
        step_size = 0.1
        theta = math.atan2((y2-y1), (x2-x1))
        #print(theta)
        boolean = True
        for element in self.obstacle_list:
            #print(element)
            x=x1
            y=y1
            while (x < x2):
                x += (step_size * math.cos(theta))
                y += (step_size * math.sin(theta))
                value = ((x-element[2])**2 + (y-element[3])** 2)
                #print(x,y,value)
                #print(element[0], element[2], element[3])
                if value <= (element[0]**2): 
                    boolean = False
                    return False 
        return True

    def reconstruct_path(self):
        key = (self.x, self.z)
        result_queue = deque([])
        final_list = deque([])
        final_list.append(key)
        result_queue.appendleft(self.rrt_graph.parents[key])
        while result_queue and self.rrt_graph.parents[key] is not None:
            key = result_queue.pop()
            final_list.appendleft(key)
            result_queue.appendleft(self.rrt_graph.parents[key])
        print(final_list)
        return final_list


