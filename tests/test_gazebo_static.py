import math
import unittest
import xml.etree.ElementTree as ET
from examples.gazebo_static import build_scene, ROOT
from dock_planner import DockPlanner, PlannerConfig, Pose2D
from dock_planner.vrx_reference import vrx_dock_2022_bay_to_dockspec
from dock_planner.wamv_collision import load_collision_primitives, footprint_bounds


class GazeboStaticTests(unittest.TestCase):
    def test_markers_match_planner(self):
        for tx, ty, yaw in ((-480,190,1), (12,-7,math.pi/2)):
            sdf, data = build_scene((tx,ty,yaw))
            models = {m.get('name'):m for m in sdf.findall('./world/model')}
            for index in (1,2,3):
                dock = vrx_dock_2022_bay_to_dockspec(tx,ty,yaw,index)
                result = DockPlanner(PlannerConfig(4,2)).plan(Pose2D(-500,180,1),dock)
                for label,p in zip(('approach','entrance','stop'),result.waypoints[1:]):
                    values = list(map(float,models[f'bay_{index}_{label}'].findtext('pose').split()))
                    self.assertAlmostEqual(values[0],p.x)
                    self.assertAlmostEqual(values[1],p.y)
                    self.assertAlmostEqual(values[5],p.yaw)
                self.assertEqual(data['bays'][index-1]['status'],'SUCCESS')

    def test_static_visuals_and_wamv_isolated(self):
        sdf,_ = build_scene()
        self.assertEqual([i.findtext('uri') for i in sdf.findall('./world/include')],
                         ['model://dock_2022_base'])
        self.assertEqual(len(sdf.findall('.//collision')), 225)
        for model in sdf.findall('./world/model'):
            self.assertEqual(model.findtext('static'),'true')
        xml = ET.tostring(sdf,encoding='unicode').lower()
        for forbidden in ('thruster','scoring','ros2','performer'):
            self.assertNotIn(forbidden,xml)
        self.assertEqual(sum('_wamv_' in m.get('name', '') for m in sdf.findall('./world/model')), 9)
        self.assertEqual(len(sdf.findall(".//collision[@name='left_float']")), 9)
        self.assertEqual(len(load_collision_primitives()), 25)

    def test_real_wamv_footprint_clearance(self):
        xmin, xmax, ymin, ymax = footprint_bounds()
        self.assertLess(xmin, -2.3)
        self.assertGreater(xmax, 2.3)
        # Check every bay and both in-bay states in the dock-local frame.  The
        # planner aligns the WAM-V yaw with entrance_heading, so this frame is
        # also the footprint frame used by the URDF collision primitives.
        for index in (1, 2, 3):
            dock = vrx_dock_2022_bay_to_dockspec(-480, 190, 1.0, index)
            result = DockPlanner(PlannerConfig(4, 2)).plan(Pose2D(-500, 180, 1), dock)
            self.assertEqual(result.status, 'SUCCESS')
            for state, waypoint_index in (('entrance', 2), ('stop', 3)):
                pose = result.waypoints[waypoint_index]
                side_clearance = min((3.0 + ymin) - 1.0, 5.0 - (3.0 + ymax))
                self.assertGreater(side_clearance, 0.7)
                self.assertLess(side_clearance, 0.9)
                self.assertAlmostEqual(pose.yaw, dock.entrance_heading)
            # At stop local x=1, the complete footprint remains before back wall x=5.
            self.assertLess(1 + xmax, 5)

    def test_heading_line_and_entrance_width(self):
        sdf,data = build_scene()
        models = {m.get('name'):m for m in sdf.findall('./world/model')}
        for b in data['bays']:
            i = b['bay_index']
            arrow = list(map(float,models[f'bay_{i}_heading'].findtext('pose').split()))
            e = b['poses']['entrance']
            self.assertAlmostEqual(arrow[0]-e['x'],math.cos(e['yaw']))
            self.assertAlmostEqual(arrow[1]-e['y'],math.sin(e['yaw']))
            self.assertAlmostEqual(arrow[5],e['yaw'])
            size = models[f'bay_{i}_entrance_line'].findtext('./link/visual/geometry/box/size')
            self.assertAlmostEqual(float(size.split()[0]),4)

    def test_generated_artifact_matches_generator(self):
        sdf,_ = build_scene()
        actual = ET.parse(ROOT/'generated/vrx_dock_static.sdf').getroot()
        self.assertEqual(ET.tostring(actual),ET.tostring(sdf))
