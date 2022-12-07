#!/usr/bin/env python3

import numpy as np
import json
import rospy
import rospkg
class Room:

    def __init__(self, name, corners):
        self.name = name 
        #form: x1y1 x2y2 (assume rectangular)
        self.corners = corners
        self.doorways = {}

    def isin(self, x, y):
        x1 = min(self.corners[0][0], self.corners[1][0])
        x2 = max(self.corners[0][0], self.corners[1][0])
        y1 = min(self.corners[0][1], self.corners[1][1])
        y2 = max(self.corners[0][1], self.corners[1][1])
        #print(f"{x1} < {x} < {x2}, {y1} < {y} < {y2}")
        return (x1 < x < x2) and (y1 < y < y2)
    #def __str__(self):
    #    return self.name
    
    #def __repr__(self):
    #    return self.name


    def to_json(self):
        return {
            "name": self.name,
            "corners": self.corners,
            "doorways": self.doorways,
        }

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name

class Graph:

    def __init__(self):

        self.size = 0
        self.adjLists = {}

    def getRoom(self,name):
        for room in self.adjLists.keys():
            #print(room.name)
            if room.name == name:
                return room
        return None

    def localise(self, x, y):
        for room in self.adjLists.keys():
            if room.isin(x, y):
                return room
        return None

    def hasVertex(self, vertex):
        for v in self.adjLists:
            if vertex.name == v.name:
                return True
        return False

    #links 2 vertex together
    def addVertex(self, vertex):
        if not self.hasVertex(vertex):
            self.size +=1
            self.adjLists[vertex] = []
            print(self.adjLists)
            return True
        else:
            return False

    def addEdge(self, u, v, u_door_pos, v_door_pos):
        u = self.getRoom(u)
        v = self.getRoom(v)
        if not u or not v:
            return False
        u.doorways[v] = u_door_pos
        self.adjLists[u].append(v)
        v.doorways[u] = v_door_pos
        self.adjLists[v].append(u)
        #print(self.adjLists)
        return True

    #as of now graph does not check for consistency
    def check_consistency(self):
        pass

    def dfs(self, u, v, visited=None):
        """
        For now, simple dfs. In future, extend to Dijkstra's.
        """
        if visited is None:
            visited = set()
        visited.add(u)
        if u == v:
            return visited
        #print("DFS ", u.name, self.getRoom(u))
        #print(self.adjLists[self.getRoom(u)])
        for x in self.adjLists[u]:
            if not x in visited:
                self.dfs(x, v, visited=visited)
        return visited
    
    def points_from_path(self, path):
        points = []
        #print(list(path))
        
        #print(list(path)[:-1])
        #print(list(path)[1:])
        for u, v in zip(list(path)[:-1],list(path)[1:]):
            points.append(u.doorways[v])
            points.append(v.doorways[u])
        return points

    def output(self, name):
        with open(rospkg.RosPack().get_path('graph_room_navigation') + f"/models/{name}.json", 'w', encoding='utf-8') as fp:
            json.dump(self.__dict__, fp)


if __name__ == "__main__":
    g = Graph()
    g.addVertex(Room('room1', [[0, 1], [1,0]]))
    g.addVertex(Room('room2', [[1, 2], [2,1]]))
    g.addEdge('room1', 'room2', [69, 96], [67, 76])
    g.output("test")
