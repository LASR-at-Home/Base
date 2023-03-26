#!/usr/bin/env python3

import numpy as np
class Room:

    def __init__(self, name, corners):
        self.name = name 
        self.corners = corners
        self.doorways = {}

    def isin(self, x, y):
        x1 = min(self.corners[0][0], self.corners[1][0])
        x2 = max(self.corners[0][0], self.corners[1][0])
        y1 = min(self.corners[0][1], self.corners[1][1])
        y2 = max(self.corners[0][1], self.corners[1][1])
        return (x1 < x < x2) and (y1 < y < y2)

    def __str__(self):
        return self.name

class Graph:

    def __init__(self):

        self.size = 0
        self.adjLists = {}

    def getRoom(self,name):
        for room in self.adjLists.keys():
            if room.name == name:
                return room
        return None

    def localise(self, x, y):
        for room in self.adjLists.keys():
            if room.isin(x, y):
                print(room.name)
                return room
        return None

    def hasVertex(self, vertex):
        for v in self.adjLists:
            if vertex.name == v.name:
                return True
        return False

    def addVertex(self, vertex):
        if not self.hasVertex(vertex):
            self.size +=1
            self.adjLists[vertex] = []
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
        return True

    def dfs(self, u, v, visited=None):
        """
        For now, simple dfs. In future, extend to Dijkstra's?
        """
        if visited is None:
            visited = []
        if not u in visited:
            visited.append(u)
        if u == v:
            return visited
        for x in self.adjLists[u]:
            if not x in visited:
                self.dfs(x, v, visited=visited)
        return visited
    
    def bfs(self, u, v):
        # maintain a queue of paths
        queue = []
        # push the first path into the queue
        queue.append([u])
        while queue:
            # get the first path from the queue
            path = queue.pop(0)
            # get the last node from the path
            node = path[-1]
            # path found
            if node == v:
                return path
            # enumerate all adjacent nodes, construct a 
            # new path and push it into the queue
            for adjacent in self.adjLists[node]:
                new_path = list(path)
                new_path.append(adjacent)
                queue.append(new_path)

    def points_from_path(self, path):
        print([p.name for p in path])
        points = []
        for u, v in zip(list(path),list(path)[1:]):
            print(u.name, v.name)
            points.append(u.doorways[v])
            points.append(v.doorways[u])
        return points
