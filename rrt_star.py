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
            nearest_vertices = []
            x_new = random.uniform(0,40)
            y_new = random.uniform(0,40)
            if not self.collision_detection( x_new, y_new):
                node, cost_list = self.nearest(x_new, y_new)
                if self.collision_detection_eq(node[0], node[1], x_new, y_new):
                    self.steer(node[0], node[1], x_new, y_new)
                    for i in range(len(cost_list)):
                        if cost_list[i] < (self.r*5):
                            nearest_vertices.append(self.rrt_graph.vertices[i])
                    self.rewire(x_new, y_new, nearest_vertices)
                    if self.calc_cost(self,x_new, y_new, self.x, self.z) < self.r: 
                        self.rrt_graph.add_edge_xy(x_new, y_new, self.x, self.z)                    
                        break 
        final_tree = self.reconstruct_path()
        return final_tree,self.rrt_graph.path_costs[(self.x, self.z)]
    
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
            if cost<initial_cost and self.collision_detection_eq(x, y, elem[0], elem[1]):
                self.steer(x,y, elem[0], elem[1])
                self.rrt_graph.update_cost_xy(elem[0], elem[1])
    
    def collision_detection(self, x, y):
        for elem in self.obstacle_list:
            value = (x - elem[2])**2 + (y-elem[3])**2
            if value <= (elem[1] ** 2): 
                return True
            else:
                return False

    def collision_detection_eq(self,x1, y1, x2, y2):
        step_size = 0.1
        distance = math.hypot((x2 - x1), (y2 - y1))
        theta = math.atan2((y2-y1), (x2-x1))
        n = distance/step_size
        for obstacle in self.obstacle_list:
            i = 0
            while i < n:
                x = x1 + (step_size * math.cos(theta)*i)
                y = y1 + (step_size * math.sin(theta)*i)
                value = math.pow((x-obstacle[2]),2) + math.pow((y-obstacle[3]),2)
                if value <= math.pow(obstacle[1],2): 
                    return False
                i +=1 
        return True

    def reconstruct_path(self):
        key = (self.x, self.z)
        result_queue = deque([])
        final_list = deque([])
        final_list.append(key)
        result_queue.appendleft(self.rrt_graph.parents[key])
        while result_queue and self.rrt_graph.parents[key] is not None:
            key = result_queue.pop()
            print(key)
            final_list.appendleft(key)
            result_queue.appendleft(self.rrt_graph.parents[key])

        return final_list


'''def main():
    obstacle_list.append([4,4,5,4])
    obstacle_list.append([2,2,8,10])
    rrt = RRTStar(0, 20, 0, 10, 3000, 2, obstacle_list)
    result, cost = rrt.planning()
    print(cost)
    xarray = []
    yarray = []
    x1arr = []
    y1arr = []
    for elem in result: 
        xarray.append(elem[0])
        yarray.append(elem[1])
    for elem in rrt.rrt_graph.vertices: 
        x1arr.append(elem[0])
        y1arr.append(elem[1])
    circle1=plt.Circle((5,4),4,color='r')
    circle2 = plt.Circle((8,10),2.5,color = 'r')
    plt.gcf().gca().add_artist(circle1) 
    plt.gcf().gca().add_artist(circle2)
    plt.plot(xarray, yarray)
    #plt.plot(x1arr, y1arr)
    plt.show()
    #result2 = rrt.collision_detection_eq(0,0,8.447922063023716, 18.46243851008471)
if __name__ == "__main__":
    main()'''
