"""Generate a static Garden world; does not launch Gazebo or load VRX code."""
import json
import math
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET

from dock_planner import DockPlanner, PlannerConfig, PlanStatus, Pose2D
from dock_planner.vrx_reference import VRX_REFERENCE_GEOMETRY, vrx_dock_2022_bay_to_dockspec
from dock_planner.wamv_collision import load_collision_primitives

ROOT = Path(__file__).resolve().parents[1]
WORLD_NAME = 'vrx_dock_static'
COLORS = {'approach': '1 0.4 0 1', 'entrance': '0 1 1 1', 'stop': '0.1 1 0.1 1'}
WAMV_MESH = 'model://WAM-V-Base/mesh/WAM-V-Base.dae'


def child(parent, tag, text):
    node = ET.SubElement(parent, tag)
    node.text = str(text)
    return node


def visual_model(world, name, x, y, z, yaw, shape, size, color):
    model = ET.SubElement(world, 'model', name=name)
    child(model, 'static', 'true')
    child(model, 'pose', f'{x:.12f} {y:.12f} {z} 0 0 {yaw:.12f}')
    link = ET.SubElement(model, 'link', name='visual_link')
    visual = ET.SubElement(link, 'visual', name='visual')
    geom = ET.SubElement(visual, 'geometry')
    element = ET.SubElement(geom, shape)
    child(element, 'radius' if shape == 'sphere' else 'size', size)
    material = ET.SubElement(visual, 'material')
    child(material, 'ambient', color)
    child(material, 'diffuse', color)
    child(visual, 'cast_shadows', 'false')


def line(world, name, a, b, color, width=0.06, z=0.64):
    dx, dy = b[0] - a[0], b[1] - a[1]
    visual_model(world, name, (a[0]+b[0])/2, (a[1]+b[1])/2, z,
                 math.atan2(dy, dx), 'box', f'{math.hypot(dx,dy)} {width} 0.04', color)


def wamv_model(world, name, pose):
    """Static WAM-V visual plus the full URDF collision geometry.

    This deliberately omits all thrusters, sensors and plugins. The collision
    Every primitive follows the personal VRX WAM-V base URDF.
    """
    x, y, yaw = pose['x'], pose['y'], pose['yaw']
    model = ET.SubElement(world, 'model', name=name)
    child(model, 'static', 'true')
    child(model, 'pose', f'{x:.12f} {y:.12f} 0.15 0 0 {yaw:.12f}')
    link = ET.SubElement(model, 'link', name='wamv_base_link')
    visual = ET.SubElement(link, 'visual', name='wamv_visual')
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    child(mesh, 'uri', WAMV_MESH)
    material = ET.SubElement(visual, 'material')
    child(material, 'ambient', '0.75 0.75 0.78 1')
    child(material, 'diffuse', '0.85 0.85 0.88 1')
    child(material, 'specular', '0.3 0.3 0.3 1')
    # Copy every collision primitive from the personal VRX WAM-V base URDF.
    for primitive in load_collision_primitives():
        collision = ET.SubElement(link, 'collision', name=primitive.name)
        x, y, z = primitive.xyz
        r, p, yy = primitive.rpy
        child(collision, 'pose', f'{x} {y} {z} {r} {p} {yy}')
        geom = ET.SubElement(collision, 'geometry')
        if primitive.kind == 'cylinder':
            cyl = ET.SubElement(geom, 'cylinder')
            child(cyl, 'radius', str(primitive.dimensions[0]))
            child(cyl, 'length', str(primitive.dimensions[1]))
        else:
            box = ET.SubElement(geom, 'box')
            child(box, 'size', ' '.join(str(v) for v in primitive.dimensions))


def build_scene(dock_pose=(-480.0, 190.0, 1.0)):
    tx, ty, yaw = dock_pose
    config = PlannerConfig(4.0, 2.0)
    usv = Pose2D(-500, 180, 1.0)  # API input only; no vehicle is spawned.
    planner = DockPlanner(config)
    records = []
    for index in (1, 2, 3):
        dock = vrx_dock_2022_bay_to_dockspec(tx, ty, yaw, index)
        result = planner.plan(usv, dock)
        if result.status != PlanStatus.SUCCESS:
            raise ValueError(result.reason)
        records.append({'bay_index': index, 'dock_spec': asdict(dock),
                        'status': result.status.value,
                        'poses': dict(zip(('approach','entrance','stop'),
                                         map(asdict, result.waypoints[1:])))})
    # Camera center follows generated marker positions, not hardcoded world coordinates.
    points = [p for r in records for p in r['poses'].values()]
    cx = sum(p['x'] for p in points)/len(points)
    cy = sum(p['y'] for p in points)/len(points)
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name=WORLD_NAME)
    child(world, 'gravity', '0 0 0')
    for filename, name in (('physics','Physics'), ('user-commands','UserCommands'),
                           ('scene-broadcaster','SceneBroadcaster')):
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system',
                      name=f'gz::sim::systems::{name}')
    scene = ET.SubElement(world, 'scene')
    child(scene, 'ambient', '0.8 0.8 0.8 1')
    child(scene, 'background', '0.15 0.2 0.3 1')
    gui = ET.SubElement(world, 'gui', fullscreen='false')
    view = ET.SubElement(gui, 'plugin', filename='MinimalScene', name='3D View')
    child(view, 'engine', 'ogre2')
    child(view, 'scene', 'scene')
    child(view, 'camera_pose', f'{cx} {cy} 27 0 1.5707963267948966 0')
    for plugin in ('GzSceneManager', 'InteractiveViewControl', 'EntityContextMenuPlugin',
                   'EntityTree', 'VisualizationCapabilities', 'Screenshot'):
        node = ET.SubElement(gui, 'plugin', filename=plugin, name=plugin)
        if plugin != 'EntityTree':
            settings = ET.SubElement(node, 'gz-gui')
            for key, kind, value in (('state','string','floating'), ('width','double','5'),
                                     ('height','double','5'), ('showTitleBar','bool','false')):
                prop = ET.SubElement(settings, 'property', key=key, type=kind)
                prop.text = value
    light = ET.SubElement(world, 'light', type='directional', name='sun')
    child(light, 'pose', f'{cx} {cy} 30 0 0 0')
    child(light, 'diffuse', '0.9 0.9 0.9 1')
    child(light, 'direction', '-0.3 0.2 -1')
    include = ET.SubElement(world, 'include')
    child(include, 'uri', 'model://dock_2022_base')
    child(include, 'name', 'vrx_reference_dock')
    child(include, 'pose', f'{tx} {ty} 0 0 0 {yaw}')
    visual_model(world, 'reference_water', cx, cy, -0.12, 0, 'box',
                 '70 70 0.04', '0.03 0.12 0.22 1')
    for record in records:
        prefix = f"bay_{record['bay_index']}"
        poses = record['poses']
        for label, p in poses.items():
            visual_model(world, f'{prefix}_{label}', p['x'], p['y'], 0.7,
                         p['yaw'], 'sphere', '0.25', COLORS[label])
            wamv_model(world, f'{prefix}_wamv_{label}', p)
        a, e, stop = (poses[k] for k in ('approach','entrance','stop'))
        line(world, prefix+'_path', (a['x'],a['y']), (stop['x'],stop['y']), '1 1 1 1')
        dx, dy = math.cos(e['yaw']), math.sin(e['yaw'])
        half = record['dock_spec']['bay_width']/2
        line(world, prefix+'_entrance_line',
             (e['x']-dy*half,e['y']+dx*half), (e['x']+dy*half,e['y']-dx*half), '0 1 1 1')
        tip = (e['x']+2*dx,e['y']+2*dy)
        line(world, prefix+'_heading', (e['x'],e['y']), tip, '1 1 0 1', z=1.02)
        for sign in (-1,1):
            base = (tip[0]-0.5*dx+sign*0.25*(-dy),tip[1]-0.5*dy+sign*0.25*dx)
            line(world, prefix+f'_arrow_{sign}', base, tip, '1 1 0 1', z=1.02)
    metadata = {'geometry_source': VRX_REFERENCE_GEOMETRY,
                'notice': 'VRX reference dimensions, NOT RobotX 2026 official dimensions.',
                'world_name': WORLD_NAME, 'dock_world_pose': {'x':tx,'y':ty,'yaw':yaw},
                'planner_config': asdict(config), 'synthetic_usv_input_not_spawned': asdict(usv),
                'units': {'position':'m','yaw':'rad'}, 'bays': records}
    metadata['wamv_static'] = {
        'instances': 9,
        'states_per_bay': ['approach', 'entrance', 'stop'],
        'collision_geometry': 'all 25 primitives copied from personal VRX wamv_base.urdf',
        'control_plugins': False,
        'vehicle_motion': False,
    }
    ET.indent(sdf, space='  ')
    return sdf, metadata


def main():
    sdf, metadata = build_scene()
    output = ROOT/'generated'
    output.mkdir(exist_ok=True)
    ET.ElementTree(sdf).write(output/'vrx_dock_static.sdf', encoding='utf-8', xml_declaration=True)
    (output/'vrx_dock_static.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(f'Generated {output}/vrx_dock_static.sdf and .json')


if __name__ == '__main__':
    main()
