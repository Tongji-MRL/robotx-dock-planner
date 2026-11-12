import json
import time
import unittest
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from usv_only_task3.node import UsvOnlyTask3Executor
from usv_only_task3.flow import UsvHealth, UsvOnlyTask3State
from dock_planner import GpsOrigin, Pose2D, NEDPoint


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
                     'checkpoint_pub', 'release_pub', 'mode_pub', 'result_pub'):
            setattr(self.node, name, Recorder())
        now = time.time()
        self.node.manager.begin((NEDPoint(0, 0, 0), NEDPoint(20, 0, 0),
                                 NEDPoint(20, 10, 0), NEDPoint(0, 10, 0)),
                                Pose2D(1, 1, 0), UsvHealth(now, 'robotx_rtk_ned'), now=now)
        self.node._rtk_origin = GpsOrigin(31.0, 121.0, 5.0)

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

    def test_official_gps_zone_is_classified_locally_and_all_clear_resumes(self):
        before = self.node.manager.state
        self.send({'run_id': 1, 'command': 'TASK4_KEEP_OUT_ZONE',
                   'parameters': {'official_command_seq': 1, 'vehicle_type': 1,
                                  'center': {'latitude': 31.0, 'longitude': 121.0},
                                  'radius_m': 2.0}})
        self.assertEqual(self.node.manager.state, before)
        self.assertEqual(self.node.manager.status()['keep_out_mode'], 'EXITING')
        self.assertIsNotNone(self.node.manager.current_goal)
        self.assertFalse(self.node.release_pub.messages[-1].data)
        self.send({'run_id': 1, 'command': 'TASK4_ALL_CLEAR',
                   'parameters': {'official_command_seq': 2, 'vehicle_type': 1}})
        self.assertEqual(self.node.manager.state, before)
        self.assertIsNotNone(self.node.manager.current_goal)

    def test_agreed_task4_begin_and_resume_apply_zone_and_ack_twice(self):
        before = self.node.manager.state
        self.send({
            'vehicle': 'usv', 'task': 'TASK4', 'progress': None, 'error': None,
            'command': {
                'type': 'TASK4_BEGIN', 'command_seq': 100,
                'center': {'latitude': 31.00001, 'longitude': 121.00001},
                'radius_m': 5.0, 'vehicle_type': 'USV',
            },
        })
        self.assertEqual(self.node.manager.state, before)
        self.assertIsNotNone(self.node.manager.navigation_constraint.zone)
        self.assertEqual(json.loads(self.node.result_pub.messages[-1].data), {
            'incident_ack': {'command_seq': 100},
        })

        self.send({
            'vehicle': 'usv', 'task': 'TASK4', 'progress': None, 'error': None,
            'command': 'resume',
        })
        self.assertIsNone(self.node.manager.navigation_constraint.zone)
        self.assertEqual(len(self.node.result_pub.messages), 2)
        self.assertEqual(json.loads(self.node.result_pub.messages[-1].data), {
            'incident_ack': {'command_seq': 100},
        })

        # A delayed duplicate BEGIN is acknowledged but must not recreate the
        # already-cleared keep-out zone.
        self.send({
            'vehicle': 'usv', 'task': 'TASK4',
            'command': {
                'type': 'TASK4_BEGIN', 'command_seq': 100,
                'center': {'latitude': 31.00001, 'longitude': 121.00001},
                'radius_m': 5.0, 'vehicle_type': 'USV',
            },
        })
        self.assertIsNone(self.node.manager.navigation_constraint.zone)
        self.assertEqual(len(self.node.result_pub.messages), 3)

    def test_task4_resume_before_begin_is_rejected_without_ack(self):
        self.send({'vehicle': 'usv', 'task': 'TASK4', 'command': 'resume'})
        self.assertEqual(self.node.result_pub.messages, [])

    def test_invalid_constraint_stops_new_targets(self):
        self.node._on_navigation_constraint(String(data=json.dumps({'run_id': 1})))
        self.assertIsNone(self.node.manager.current_goal)
        self.assertEqual(self.node.manager.navigation_hold_reason, 'INVALID_NAVIGATION_CONSTRAINT')
        self.assertTrue(self.node.release_pub.messages[-1].data)

    def test_standalone_gps_is_accepted_but_no_fix_is_rejected(self):
        ordinary = NavSatFix()
        ordinary.status.status = NavSatStatus.STATUS_FIX
        ordinary.latitude = 31.00001
        ordinary.longitude = 121.00001
        ordinary.altitude = 5.0
        ordinary.header.stamp.sec = int(time.time())
        self.node._on_mavros_global_fix(ordinary)
        accepted = self.node._mavros_fix
        self.assertIsNotNone(accepted)

        no_fix = NavSatFix()
        no_fix.status.status = NavSatStatus.STATUS_NO_FIX
        no_fix.latitude = 32.0
        no_fix.longitude = 122.0
        self.node._on_mavros_global_fix(no_fix)
        self.assertEqual(self.node._mavros_fix, accepted)

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
