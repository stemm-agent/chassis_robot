#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from stemm_coverage_explorer.selector import GridMap, Pose2D, select_coverage_goal


class CoverageExplorer(Node):
    def __init__(self):
        super().__init__('stemm_coverage_explorer')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('nav_state_topic', '/stemm/nav_state')
        self.declare_parameter('goal_topic', '/stemm/nav_goal')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('map_timeout_sec', 10.0)
        self.declare_parameter('costmap_timeout_sec', 3.0)
        self.declare_parameter('goal_interval_sec', 5.0)
        self.declare_parameter('min_goal_distance', 0.6)
        self.declare_parameter('max_goal_distance', 3.0)
        self.declare_parameter('obstacle_clearance', 0.38)
        self.declare_parameter('known_clearance', 0.20)
        self.declare_parameter('corridor_clearance', 0.28)
        self.declare_parameter('frontier_radius', 0.7)
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('potential_scale', 1.2)
        self.declare_parameter('clearance_scale', 0.8)
        self.declare_parameter('lookahead_scale', 0.35)
        self.declare_parameter('bad_goal_radius', 0.55)
        self.declare_parameter('blocked_place_radius', 0.55)
        self.declare_parameter('bad_ttl_sec', 240.0)
        self.declare_parameter('finish_save_delay_sec', 30.0)

        self.active = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').get_parameter_value().string_value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').get_parameter_value().string_value
        self.nav_state_topic = self.get_parameter('nav_state_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.map_timeout_sec = self.get_parameter('map_timeout_sec').get_parameter_value().double_value
        self.costmap_timeout_sec = self.get_parameter('costmap_timeout_sec').get_parameter_value().double_value
        self.goal_interval_sec = self.get_parameter('goal_interval_sec').get_parameter_value().double_value
        self.min_goal_distance = self.get_parameter('min_goal_distance').get_parameter_value().double_value
        self.max_goal_distance = self.get_parameter('max_goal_distance').get_parameter_value().double_value
        self.obstacle_clearance = self.get_parameter('obstacle_clearance').get_parameter_value().double_value
        self.known_clearance = self.get_parameter('known_clearance').get_parameter_value().double_value
        self.corridor_clearance = self.get_parameter('corridor_clearance').get_parameter_value().double_value
        self.frontier_radius = self.get_parameter('frontier_radius').get_parameter_value().double_value
        self.min_frontier_cells = self.get_parameter('min_frontier_cells').get_parameter_value().integer_value
        self.gain_scale = self.get_parameter('gain_scale').get_parameter_value().double_value
        self.potential_scale = self.get_parameter('potential_scale').get_parameter_value().double_value
        self.clearance_scale = self.get_parameter('clearance_scale').get_parameter_value().double_value
        self.lookahead_scale = self.get_parameter('lookahead_scale').get_parameter_value().double_value
        self.bad_goal_radius = self.get_parameter('bad_goal_radius').get_parameter_value().double_value
        self.blocked_place_radius = self.get_parameter('blocked_place_radius').get_parameter_value().double_value
        self.bad_ttl_sec = self.get_parameter('bad_ttl_sec').get_parameter_value().double_value
        self.finish_save_delay_sec = self.get_parameter('finish_save_delay_sec').get_parameter_value().double_value

        self.last_map = None
        self.last_map_time = 0.0
        self.last_global_costmap = None
        self.last_global_costmap_time = 0.0
        self.last_local_costmap = None
        self.last_local_costmap_time = 0.0
        self.nav_state = {}
        self.bad_places = []
        self.last_goal_time = 0.0
        self.no_goal_since = None
        self.save_requested = False
        self.status = 'idle'

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.state_pub = self.create_publisher(String, '/stemm/coverage_state', 10)
        self.save_client = self.create_client(Trigger, '/stemm/save_map')

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_subscription(OccupancyGrid, self.global_costmap_topic, self.global_costmap_callback, 10)
        self.create_subscription(OccupancyGrid, self.local_costmap_topic, self.local_costmap_callback, 10)
        self.create_subscription(String, self.nav_state_topic, self.nav_state_callback, 10)
        self.create_service(Trigger, '/stemm/coverage_start', self.start_callback)
        self.create_service(Trigger, '/stemm/coverage_stop', self.stop_callback)

        self.create_timer(1.0, self.publish_state)
        self.create_timer(2.0, self.tick)
        self.get_logger().info('STEMM coverage explorer ready')

    def map_callback(self, msg):
        self.last_map = msg
        self.last_map_time = time.monotonic()

    def global_costmap_callback(self, msg):
        self.last_global_costmap = msg
        self.last_global_costmap_time = time.monotonic()

    def local_costmap_callback(self, msg):
        self.last_local_costmap = msg
        self.last_local_costmap_time = time.monotonic()

    def nav_state_callback(self, msg):
        try:
            self.nav_state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.remember_failed_state()

    def start_callback(self, request, response):
        del request
        self.bad_places = []
        self.no_goal_since = None
        self.save_requested = False
        self.last_goal_time = 0.0
        self.active = True
        self.status = 'active'
        response.success = True
        response.message = 'coverage exploration started'
        return response

    def stop_callback(self, request, response):
        del request
        self.active = False
        self.status = 'stopped'
        response.success = True
        response.message = 'coverage exploration stopped'
        return response

    def remember_failed_state(self):
        for goal in self.nav_state.get('failed_goals') or []:
            if goal.get('x') is not None and goal.get('y') is not None:
                self.add_bad_place(float(goal['x']), float(goal['y']), float(goal.get('radius', self.bad_goal_radius)))

        for place in self.nav_state.get('blocked_places') or []:
            if place.get('x') is not None and place.get('y') is not None:
                self.add_bad_place(float(place['x']), float(place['y']), float(place.get('radius', self.blocked_place_radius)))

        nav_state = str(self.nav_state.get('navigation_state', ''))
        goal_failed = nav_state.startswith('failed') or nav_state in (
            'goal_rejected',
            'goal_unreachable',
            'stuck_waiting_new_goal',
        )
        place_failed = 'stuck' in nav_state or 'blocked' in nav_state
        if not goal_failed and not place_failed:
            return

        goal = self.nav_state.get('last_goal')
        if goal and goal.get('x') is not None and goal.get('y') is not None:
            self.add_bad_place(float(goal['x']), float(goal['y']), self.bad_goal_radius)

        if place_failed:
            pose = self.nav_state.get('pose')
            if pose and pose.get('x') is not None and pose.get('y') is not None:
                self.add_bad_place(float(pose['x']), float(pose['y']), self.blocked_place_radius)

    def add_bad_place(self, x, y, radius):
        now = time.monotonic()
        for index, (bx, by, _, old_radius) in enumerate(self.bad_places):
            if math.hypot(x - bx, y - by) < 0.3:
                self.bad_places[index] = (x, y, now, max(radius, old_radius))
                return
        self.bad_places.append((x, y, now, radius))

    def prune_bad_places(self):
        now = time.monotonic()
        self.bad_places = [place for place in self.bad_places if now - place[2] < self.bad_ttl_sec]

    def map_is_fresh(self):
        return self.last_map is not None and time.monotonic() - self.last_map_time <= self.map_timeout_sec

    def costmap_is_fresh(self, stamp):
        return stamp > 0.0 and time.monotonic() - stamp <= self.costmap_timeout_sec

    def nav_is_busy(self):
        nav_state = str(self.nav_state.get('navigation_state', ''))
        mode = str(self.nav_state.get('mode', ''))
        recovery_states = (
            'sending_goal',
            'executing',
            'blocked_waiting_clearance',
            'turning_out_of_blocked_place',
            'turning_out_of_stuck_place',
            'stuck_waiting_new_goal',
        )
        return mode in ('navigating', 'safety_stop') or nav_state in recovery_states

    def nav_manager_is_ready(self):
        return bool(self.nav_state.get('nav2_ready'))

    def base_is_charging(self):
        return bool(self.nav_state.get('base_charging'))

    def current_pose(self):
        pose = self.nav_state.get('pose')
        if not pose or pose.get('x') is None or pose.get('y') is None:
            return None
        return Pose2D(float(pose['x']), float(pose['y']))

    def to_grid_map(self, msg):
        info = msg.info
        return GridMap(
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            msg.data,
        )

    def fresh_global_costmap(self):
        if self.last_global_costmap is None or not self.costmap_is_fresh(self.last_global_costmap_time):
            return None
        if self.last_global_costmap.header.frame_id and self.last_global_costmap.header.frame_id != self.global_frame:
            return None
        return self.to_grid_map(self.last_global_costmap)

    def fresh_local_costmap(self):
        if self.last_local_costmap is None or not self.costmap_is_fresh(self.last_local_costmap_time):
            return None
        if self.last_local_costmap.header.frame_id and self.last_local_costmap.header.frame_id != self.global_frame:
            return None
        return self.to_grid_map(self.last_local_costmap)

    def tick(self):
        if not self.active:
            return
        if not self.map_is_fresh():
            self.status = 'waiting_for_map'
            return
        pose = self.current_pose()
        if pose is None:
            self.status = 'waiting_for_pose'
            return
        if not self.nav_manager_is_ready():
            self.no_goal_since = None
            self.status = 'waiting_for_nav2'
            return
        if self.base_is_charging():
            self.no_goal_since = None
            self.status = 'waiting_for_charging_disconnect'
            return
        if self.nav_is_busy():
            self.status = 'waiting_for_navigation'
            return
        if time.monotonic() - self.last_goal_time < self.goal_interval_sec:
            return

        self.prune_bad_places()
        goal = select_coverage_goal(
            self.to_grid_map(self.last_map),
            pose,
            bad_places=[(x, y, radius) for x, y, _, radius in self.bad_places],
            min_distance=self.min_goal_distance,
            max_distance=self.max_goal_distance,
            obstacle_clearance=self.obstacle_clearance,
            known_clearance=self.known_clearance,
            corridor_clearance=self.corridor_clearance,
            frontier_radius=self.frontier_radius,
            bad_radius=self.blocked_place_radius,
            global_costmap=self.fresh_global_costmap(),
            local_costmap=self.fresh_local_costmap(),
            min_frontier_cells=self.min_frontier_cells,
            gain_scale=self.gain_scale,
            potential_scale=self.potential_scale,
            clearance_scale=self.clearance_scale,
            lookahead_scale=self.lookahead_scale,
        )

        if goal is None:
            self.status = 'waiting_for_safe_frontier'
            if self.no_goal_since is None:
                self.no_goal_since = time.monotonic()
            elif not self.save_requested and time.monotonic() - self.no_goal_since >= self.finish_save_delay_sec:
                self.request_save_map()
            return

        self.no_goal_since = None
        self.save_requested = False
        self.publish_goal(goal.x, goal.y, pose)
        self.status = f'goal_sent:{goal.x:.2f},{goal.y:.2f}'
        self.last_goal_time = time.monotonic()

    def publish_goal(self, x, y, pose):
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        yaw = math.atan2(y - pose.y, x - pose.x)
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_pub.publish(goal)
        self.get_logger().info(f'coverage goal: {x:.2f}, {y:.2f}')

    def request_save_map(self):
        if not self.save_client.wait_for_service(timeout_sec=0.1):
            self.status = 'finished_save_service_unavailable'
            return
        self.save_requested = True
        self.save_client.call_async(Trigger.Request())
        self.status = 'finished_save_requested'

    def publish_state(self):
        payload = {
            'active': self.active,
            'status': self.status,
            'map_available': self.map_is_fresh(),
            'global_costmap_available': self.fresh_global_costmap() is not None,
            'local_costmap_available': self.fresh_local_costmap() is not None,
            'base_charging': self.base_is_charging(),
            'bad_place_count': len(self.bad_places),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
