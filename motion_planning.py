import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global

from project_utils import calculate_waypoints, three_tuple, save_path, show_grid

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal):
        super().__init__(connection)

        if goal is not None:
            self.goal = np.array(goal)
        else:
            self.goal = None

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if abs(self.local_position[2]) < 1.03*self.goal[-1] and abs(self.local_position[2]) > 0.97*self.goal[-1]:  # not always possible to get to 0 alt
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if not self.armed and not self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        if len(self.waypoints) < 2:
            raise RuntimeError("The path data given indicates that either the path is empty or consists only of the current starting point.  Exiting...")
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open("colliders.csv") as f:
            lat, lon = f.readline().split(",")
        lat0 = float(lat.split(" ")[-1])
        lon0 = float(lon.split(" ")[-1])
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        g_home = self.global_position
        print("Current global position: {}".format(g_home)) 
 
        # TODO: convert to current local position using global_to_local()
        l_home = global_to_local(g_home, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home,
                                                                         self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        if self.goal is None:
            # XXX I used this logic path to explore the space and grab some candidate
            # goal points.  NOTE: THIS LOGIC SHOULD NOT BE EXERCISED IN NORMAL EXECUTION
            #show_grid(grid)
            # found the following interesting goals
            interesting_goals = [[743.0, 540.0, -3.0],
                                 [325.0, 825.0, -3.0],
                                 [124.0, 807.0, -3.0],
                                 [800.0, 195.7, -3.0]]
            for l in interesting_goals:
                l_np = np.array(l)
                l_np[0] += north_offset
                l_np[1] += east_offset
                g_np = local_to_global(l_np, self.global_home)
                print('##### {}'.format(g_np))
                
            # exit when done
            self.waypoints = []
            self.send_waypoints()
            return

        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.ceil(l_home[0] - north_offset)),
                      int(np.ceil(l_home[1] - east_offset)))
        
        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        l_goal = global_to_local(self.goal, self.global_home)
        grid_goal = (int(np.ceil(l_goal[0] - north_offset)),
                     int(np.ceil(l_goal[1] - east_offset)))

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        self.waypoints = calculate_waypoints(self.global_position, self.goal, self.global_home, data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        grid_wp = [(p[0] - north_offset, p[1] - east_offset) for p in self.waypoints]
        save_path(grid, grid_wp, grid_start, grid_goal) 
        # Convert path to waypoints
        #waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        #self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    # add ability to set goal from command-line
    parser.add_argument('--goal', type=three_tuple, help='(lon, lat, alt) goal position (comma-separated)')
    # interesting goals come from looking at the grid (see show_grid function)
    # I chose an altitude of 3.0 so that the vehicle could get through landing stage
    # even it was still above the ground.  As long as height above ground for each lon,lat
    # is unknown at runtime, this is the best I could think of given the data I did observe.
    # interesting points I found (in geodetic coordinates) were:
    ##### [-122.397        37.7953        3.        ]
    ##### [-122.39634302   37.79632331    3.        ]
    ##### [-122.39313684   37.79253936    3.        ]
    ##### [-122.39335608   37.79072886    3.        ]
    ##### [-122.40024939   37.796857      3.        ]
    args = parser.parse_args()

    # XXX comment out this check if you want to pull up a plot to identify goal points
    if not args.goal:
        raise RuntimeError("User must specify a goal in geodetic coordinates")

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=300)
    drone = MotionPlanning(conn, args.goal)

    time.sleep(1)

    drone.start()
