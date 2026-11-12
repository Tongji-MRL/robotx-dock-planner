import json
import time
import unittest
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from usv_only_task3.node import UsvOnlyTask3Executor
from usv_only_task3.flow import UsvHealth, UsvOnlyTask3State
from dock_planner import Pose2D, NEDPoint


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Task4NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = UsvOnlyTask3Executor()
        # Capture output without putting test targets onto ROS topics.
        for name in ('progress_pub', 'preview_pub', 'goto_pub', 'state_pub',
                     'checkpoint_pub', 'release_pub', 'mode_pub'):
            setattr(self.node, name, Recorder())
        now = time.time()
        self.node.manager.begin((NEDPoint(0, 0, 0), NEDPoint(20, 0, 0),
                                 NEDPoint(20, 10, 0), NEDPoint(0, 10, 0)),
                                Pose2D(1, 1, 0), UsvHealth(now, 'robotx_rtk_ned'), now=now)

    def tearDown(self):
        self.node.destroy_node()

    def send(self, value):
        msg = String(); msg.data = json.dumps(value)
        self.node._on_command(msg)

    def test_core_checkpoint_then_resume_does_not_replay_old_done(self):
        self.send({'task': 'TASK4', 'command': {'type': 'TASK4_BEGIN'}})
        checkpoint = json.loads(self.node.checkpoint_pub.messages[-1].data)
        self.assertEqual(checkpoint['resume_phase'], 'EXPLORING')
        self.assertFalse(checkpoint['motion_stopped_confirmed'])
        self.send({'task': 'TASK3', 'command': 'resume', 'checkpoint_id': checkpoint['checkpoint_id'],
                   'motion_stopped': True})
        before = self.node.manager.state
        self.node._on_goto(String(data='DONE'))
        self.assertEqual(self.node.manager.state, before)
        self.assertEqual(self.node.goto_pub.messages, [])  # dry run

    def test_inside_hold_and_all_clear_preserve_progress(self):
        before = self.node.manager.state
        self.send({'task': 'TASK4', 'command': dict(type='TASK4_KEEP_OUT', run_id=1,
            revision=1, motion_stopped=True, inside=True, frame_id='robotx_rtk_ned',
            center_ned={'north': 5, 'east': 5}, radius_m=2, clearance_m=1)})
        self.assertEqual(self.node.manager.state, before)
        self.assertIsNone(self.node.manager.current_goal)
        self.assertTrue(self.node.release_pub.messages[-1].data)
        self.send({'task': 'TASK4', 'command': dict(type='TASK4_ALL_CLEAR', run_id=1,
                   revision=2, motion_stopped=True)})
        self.assertEqual(self.node.manager.state, before)
        self.assertIsNotNone(self.node.manager.current_goal)

    def test_invalid_constraint_stops_new_targets(self):
        self.node._on_navigation_constraint(String(data=json.dumps({'run_id': 1})))
        self.assertIsNone(self.node.manager.current_goal)
        self.assertEqual(self.node.manager.navigation_hold_reason, 'INVALID_NAVIGATION_CONSTRAINT')
        self.assertTrue(self.node.release_pub.messages[-1].data)

    def test_real_goto_output_is_point_stamped_in_local_enu(self):
        now = time.time()
        self.node.dry_run = False
        self.node.enable_goto_output = True
        self.node._pose = Pose2D(1.0, 1.0, 0.0)
        self.node._health = UsvHealth(now, 'robotx_rtk_ned')
        odom = Odometry()
        odom.header.stamp.sec = int(now)
        odom.header.stamp.nanosec = int((now - int(now)) * 1e9)
        odom.pose.pose.position.x = 100.0
        odom.pose.pose.position.y = 200.0
        self.node._on_odom(odom)

        goal = self.node.manager.current_goal
        self.assertIsNotNone(goal)
        self.node._last_goal = None
        self.node._publish()

        target = self.node.goto_pub.messages[-1]
        self.assertIsInstance(target, PointStamped)
        self.assertEqual(target.header.frame_id, 'map')
        self.assertAlmostEqual(target.point.x, 100.0 + goal.y - 1.0)
        self.assertAlmostEqual(target.point.y, 200.0 + goal.x - 1.0)
        self.assertEqual(target.point.z, 0.0)

    def test_real_goto_output_rejects_unsynchronised_reference(self):
        now = time.time()
        self.node.dry_run = False
        self.node.enable_goto_output = True
        self.node._pose = Pose2D(1.0, 1.0, 0.0)
        self.node._health = UsvHealth(now, 'robotx_rtk_ned')
        odom = Odometry()
        odom.header.stamp.sec = int(now - 1.0)
        odom.pose.pose.position.x = 100.0
        odom.pose.pose.position.y = 200.0
        self.node._on_odom(odom)

        self.node._last_goal = None
        self.node._publish()

        self.assertEqual(self.node.goto_pub.messages, [])
        self.assertFalse(self.node._navigation_active)
