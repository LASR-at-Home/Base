#!/usr/bin/env python3
import rospy

from graph_room_navigation import Graph, Room
from graph_room_navigation.srv import AddRoom, AddRoomResponse, AddCrossing, AddCrossingResponse, PlanToRoom, PlanToRoomResponse, PlanToPoint, PlanToPointResponse
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
class GraphNavigationServer():

    def __init__(self, path=""):
        self.graph = Graph()
        
        self.add_room_srv = rospy.Service("graph_navigation_server/add_room", AddRoom, self.add_room)
        self.add_doorway_srv = rospy.Service("graph_navigation_server/add_doorway", AddCrossing, self.add_crossing)
        self.plan_to_room_srv = rospy.Service("graph_navigation_server/plan_to_room", PlanToRoom, self.plan_to_room)
        self.plan_to_point_srv = rospy.Service("graph_navigation_server/plan_to_point", PlanToPoint, self.plan_to_point)

    def add_room(self, req):
        response = AddRoomResponse()
        response.success = self.graph.addVertex(Room(req.name, [[req.a.x, req.a.y], [req.b.x, req.b.y]])) 
        return response

    def add_crossing(self, req):
        response = AddCrossingResponse()
        response.success = self.graph.addEdge(req.room1, req.room2, [req.door1.x, req.door1.y], [req.door2.x, req.door2.y])
        return response

    def plan_to_room(self, req):
        """
        Given a goal room, generate a plan to that room.
        """
        response = PlanToRoomResponse()
        #print(self.graph.dfs(self.current_room().name, req.goal_room))
        path = self.graph.points_from_path(self.graph.bfs(self.current_room(), self.graph.getRoom(req.goal_room)))
        print(path)
        response.points = [Point(p[0], p[1], 0) for p in path]
        if path:
            response.success = True
        return response

    def plan_to_point(self, req):
        """
        Given a point, generate a plan to that point.
        """
        response = PlanToPointResponse()
        path = self.graph.points_from_path(self.graph.bfs(self.current_room(), self.graph.localise(req.goal.x, req.goal.y)))
        response.points = [Point(p[0], p[1], 0) for p in path]
        if path:
            response.success = True
        return response
    
    def current_room(self):
        robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        x = robot_pose.pose.pose.position.x
        y = robot_pose.pose.pose.position.y
        return self.graph.localise(x, y)

if __name__ == "__main__":
    rospy.init_node("graph_navigation_server")
    server = GraphNavigationServer()
    rospy.spin()

