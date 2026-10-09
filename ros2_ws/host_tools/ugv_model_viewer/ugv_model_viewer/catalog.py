"""Catalog of the Wheeltec chassis URDFs from the `wheeltec_robot_urdf` extension.

The extension (downloaded into `extensions/`, see extensions/README.md) is an
unmodified copy of the vendor package and is never built: its mesh URIs
(`package://wheeltec_robot_urdf/...`) are rewritten to `file://` URIs so RViz
loads the STL files in place.
"""

from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import quote
import re
import xml.etree.ElementTree as ET

import numpy as np

MODEL_PACKAGE = 'wheeltec_robot_urdf'

# (name token, family label). First match wins, so specific tokens come first.
FAMILIES = (
    ('four_wheel_diff', '4-wheel differential'),
    ('akm', 'Ackermann'),
    ('mec', 'Mecanum'),
    ('4wd', '4WD skid-steer'),
    ('omni', 'Omni'),
    ('tank', 'Tracked'),
    ('diff', 'Differential'),
)
OTHER_FAMILY = 'Other'


def family_of(model):
    lowered = model.lower()
    for token, label in FAMILIES:
        if token in lowered:
            return label
    return OTHER_FAMILY


@dataclass
class ModelSummary:
    root_link: str
    movable_joints: list = field(default_factory=list)  # [(name, position)]
    bbox_min: np.ndarray = None  # in root link frame, metres
    bbox_max: np.ndarray = None
    track: float = None  # wheel centre to wheel centre, across (y)
    wheelbase: float = None  # wheel centre to wheel centre, along (x)
    wheel_diameter: float = None


class Catalog:

    def __init__(self, urdf_dir):
        self.urdf_dir = Path(urdf_dir).resolve()
        self.package_dir = self.urdf_dir.parent
        if not self.urdf_dir.is_dir():
            raise FileNotFoundError(
                f'URDF directory not found: {self.urdf_dir}\n'
                f'Download the {MODEL_PACKAGE} extension (see extensions/README.md).')
        family_order = [label for _, label in FAMILIES] + [OTHER_FAMILY]
        self.models = sorted(
            (p.stem for p in self.urdf_dir.glob('*.urdf')),
            key=lambda m: (family_order.index(family_of(m)), m.lower()))
        if not self.models:
            raise FileNotFoundError(f'No *.urdf files in {self.urdf_dir}')
        self._summaries = {}

    def _raw(self, model):
        if model not in self.models:
            raise KeyError(f"Unknown model '{model}'. Available: {', '.join(self.models)}")
        return (self.urdf_dir / f'{model}.urdf').read_text(encoding='utf-8')

    def description(self, model):
        """URDF text with mesh URIs pointing at the extension files on disk."""
        base = 'file://' + quote(str(self.package_dir)) + '/'
        return self._raw(model).replace(f'package://{MODEL_PACKAGE}/', base)

    def summary(self, model):
        if model not in self._summaries:
            self._summaries[model] = _summarize(self._raw(model), self.package_dir)
        return self._summaries[model]


def _floats(text, default):
    return [float(v) for v in (text or default).split()]


def _transform(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    t = np.eye(4)
    t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    t[:3, 3] = xyz
    return t


def _origin(elem):
    origin = elem.find('origin') if elem is not None else None
    if origin is None:
        return np.eye(4)
    return _transform(_floats(origin.get('xyz'), '0 0 0'), _floats(origin.get('rpy'), '0 0 0'))


def _joint_motion(joint_type, axis, position):
    t = np.eye(4)
    if position == 0.0:
        return t
    axis = np.asarray(axis) / np.linalg.norm(axis)
    if joint_type == 'prismatic':
        t[:3, 3] = axis * position
    else:  # Rodrigues rotation about the joint axis
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        t[:3, :3] = np.eye(3) + np.sin(position) * k + (1 - np.cos(position)) * k @ k
    return t


def _default_position(joint):
    # Zero, clamped into the joint limits so every link has a valid pose.
    limit = joint.find('limit')
    if joint.get('type') in ('revolute', 'prismatic') and limit is not None:
        lower = float(limit.get('lower', 0.0))
        upper = float(limit.get('upper', 0.0))
        return min(max(0.0, lower), upper)
    return 0.0


def _load_stl(path):
    data = path.read_bytes()
    if len(data) >= 84:
        count = int.from_bytes(data[80:84], 'little')
        if len(data) == 84 + 50 * count:
            record = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
            return np.frombuffer(data, record, count, 84)['v'].reshape(-1, 3).astype(float)
    text = data.decode('ascii', errors='ignore')
    nums = re.findall(r'vertex\s+(\S+)\s+(\S+)\s+(\S+)', text)
    return np.array(nums, dtype=float).reshape(-1, 3)


def _box_corners(half):
    return np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half


def _visual_points(visual, package_dir):
    """Points of one <visual> in its link frame, or None if not measurable."""
    geometry = visual.find('geometry')
    if geometry is None:
        return None
    mesh, box = geometry.find('mesh'), geometry.find('box')
    cylinder, sphere = geometry.find('cylinder'), geometry.find('sphere')
    if mesh is not None:
        uri = mesh.get('filename', '')
        prefix = f'package://{MODEL_PACKAGE}/'
        path = package_dir / uri[len(prefix):] if uri.startswith(prefix) else None
        if path is None or path.suffix.lower() != '.stl' or not path.is_file():
            return None
        points = _load_stl(path) * _floats(mesh.get('scale'), '1 1 1')
    elif box is not None:
        points = _box_corners(np.array(_floats(box.get('size'), '0 0 0')) / 2)
    elif cylinder is not None:
        r, half_len = float(cylinder.get('radius')), float(cylinder.get('length')) / 2
        points = _box_corners(np.array([r, r, half_len]))
    elif sphere is not None:
        points = _box_corners(np.full(3, float(sphere.get('radius'))))
    else:
        return None
    homogeneous = np.c_[points, np.ones(len(points))]
    return (homogeneous @ _origin(visual).T)[:, :3]


def _summarize(urdf_text, package_dir):
    robot = ET.fromstring(urdf_text)
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = robot.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    roots = [name for name in links if name not in children]
    root = 'base_link' if 'base_link' in roots else roots[0]

    summary = ModelSummary(root_link=root)
    by_parent = {}
    for joint in joints:
        by_parent.setdefault(joint.find('parent').get('link'), []).append(joint)

    # Forward kinematics at the default joint positions.
    poses, stack, wheel_joints = {root: np.eye(4)}, [root], []
    while stack:
        parent = stack.pop()
        for joint in by_parent.get(parent, []):
            child, joint_type = joint.find('child').get('link'), joint.get('type')
            position = 0.0
            if joint_type != 'fixed':
                position = _default_position(joint)
                summary.movable_joints.append((joint.get('name'), position))
            axis = _floats(joint.find('axis').get('xyz') if joint.find('axis') is not None
                           else None, '1 0 0')
            poses[child] = poses[parent] @ _origin(joint) @ _joint_motion(joint_type, axis, position)
            if 'wheel' in child.lower() and joint_type in ('continuous', 'revolute'):
                wheel_joints.append(child)
            stack.append(child)

    all_points, wheel_sizes = [], []
    for name, pose in poses.items():
        link_points = [p for p in (_visual_points(v, package_dir)
                                   for v in links[name].findall('visual')) if p is not None]
        if not link_points:
            continue
        local = np.vstack(link_points)
        if name in wheel_joints:
            wheel_sizes.append(float((local.max(axis=0) - local.min(axis=0)).max()))
        all_points.append((np.c_[local, np.ones(len(local))] @ pose.T)[:, :3])
    if all_points:
        stacked = np.vstack(all_points)
        summary.bbox_min, summary.bbox_max = stacked.min(axis=0), stacked.max(axis=0)

    if len(wheel_joints) >= 2:
        centres = np.array([poses[name][:3, 3] for name in wheel_joints])
        span = centres.max(axis=0) - centres.min(axis=0)
        summary.track = float(span[1]) if span[1] > 1e-3 else None
        summary.wheelbase = float(span[0]) if span[0] > 1e-3 else None
    if wheel_sizes:
        summary.wheel_diameter = float(np.median(wheel_sizes))
    return summary
