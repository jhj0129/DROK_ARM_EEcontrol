#!/usr/bin/env python3
import xml.etree.ElementTree as ET
from pathlib import Path
import html

SDF_PATH = Path.home() / ".gazebo/models/arm_only_clean/model.sdf"
OUT_PATH = Path.home() / "Workspace1/ros2_ws/src/arm_only_description/urdf/arm_only_clean_moveit.urdf.xacro"

PACKAGE_NAME = "arm_only_description"

def get_text(elem, path, default=None):
    found = elem.find(path)
    if found is None or found.text is None:
        return default
    return found.text.strip()

def split_pose(pose_text):
    vals = [float(x) for x in pose_text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return vals[:6]

def origin_xml(pose_text, indent):
    if pose_text is None:
        pose_text = "0 0 0 0 0 0"
    x, y, z, r, p, yw = split_pose(pose_text)
    return f'{indent}<origin xyz="{x:g} {y:g} {z:g}" rpy="{r:g} {p:g} {yw:g}" />\n'

def convert_mesh_uri(uri):
    if uri is None:
        return ""
    uri = uri.strip()
    if "meshes/" in uri:
        mesh_name = uri.split("meshes/")[-1]
        return f"package://{PACKAGE_NAME}/meshes/{mesh_name}"
    if uri.startswith("model://arm_only_clean/"):
        return uri.replace("model://arm_only_clean/", f"package://{PACKAGE_NAME}/")
    return uri

def geometry_xml(geom, indent):
    if geom is None:
        return ""

    box = geom.find("box")
    if box is not None:
        size = get_text(box, "size", "0.01 0.01 0.01")
        return (
            f"{indent}<geometry>\n"
            f"{indent}  <box size=\"{size}\" />\n"
            f"{indent}</geometry>\n"
        )

    cyl = geom.find("cylinder")
    if cyl is not None:
        radius = get_text(cyl, "radius", "0.01")
        length = get_text(cyl, "length", "0.01")
        return (
            f"{indent}<geometry>\n"
            f"{indent}  <cylinder radius=\"{radius}\" length=\"{length}\" />\n"
            f"{indent}</geometry>\n"
        )

    mesh = geom.find("mesh")
    if mesh is not None:
        uri = get_text(mesh, "uri", "")
        scale = get_text(mesh, "scale", "1 1 1")
        filename = convert_mesh_uri(uri)
        return (
            f"{indent}<geometry>\n"
            f"{indent}  <mesh filename=\"{html.escape(filename)}\" scale=\"{scale}\" />\n"
            f"{indent}</geometry>\n"
        )

    return ""

def safe_name(name):
    return html.escape(name if name else "")

tree = ET.parse(SDF_PATH)
root = tree.getroot()
model = root.find("model")

if model is None:
    raise RuntimeError("model tag not found in SDF")

lines = []
lines.append('<?xml version="1.0"?>\n')
lines.append('<robot name="arm_only_clean" xmlns:xacro="http://www.ros.org/wiki/xacro">\n\n')

# Links
for link in model.findall("link"):
    link_name = link.attrib["name"]
    lines.append(f'  <link name="{safe_name(link_name)}">\n')

    inertial = link.find("inertial")
    if inertial is not None:
        mass = get_text(inertial, "mass", "0.001")
        pose = get_text(inertial, "pose", "0 0 0 0 0 0")
        ixx = get_text(inertial, "inertia/ixx", "1e-6")
        ixy = get_text(inertial, "inertia/ixy", "0")
        ixz = get_text(inertial, "inertia/ixz", "0")
        iyy = get_text(inertial, "inertia/iyy", "1e-6")
        iyz = get_text(inertial, "inertia/iyz", "0")
        izz = get_text(inertial, "inertia/izz", "1e-6")

        lines.append("    <inertial>\n")
        lines.append(origin_xml(pose, "      "))
        lines.append(f'      <mass value="{mass}" />\n')
        lines.append(
            f'      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" '
            f'iyy="{iyy}" iyz="{iyz}" izz="{izz}" />\n'
        )
        lines.append("    </inertial>\n")

    for visual in link.findall("visual"):
        vname = visual.attrib.get("name", "")
        pose = get_text(visual, "pose", "0 0 0 0 0 0")
        geom = visual.find("geometry")

        lines.append(f'    <visual name="{safe_name(vname)}">\n')
        lines.append(origin_xml(pose, "      "))
        lines.append(geometry_xml(geom, "      "))
        lines.append("    </visual>\n")

    for collision in link.findall("collision"):
        cname = collision.attrib.get("name", "")
        pose = get_text(collision, "pose", "0 0 0 0 0 0")
        geom = collision.find("geometry")

        lines.append(f'    <collision name="{safe_name(cname)}">\n')
        lines.append(origin_xml(pose, "      "))
        lines.append(geometry_xml(geom, "      "))
        lines.append("    </collision>\n")

    lines.append("  </link>\n\n")

# Joints
for joint in model.findall("joint"):
    jname = joint.attrib.get("name", "")
    jtype = joint.attrib.get("type", "fixed")

    parent = get_text(joint, "parent")
    child = get_text(joint, "child")

    # world fixed joint는 MoveIt에서 virtual joint로 만들 것이므로 URDF에는 넣지 않음
    if parent == "world" or child is None or parent is None:
        continue

    pose = get_text(joint, "pose", "0 0 0 0 0 0")

    if jtype == "revolute2":
        jtype = "revolute"

    lines.append(f'  <joint name="{safe_name(jname)}" type="{jtype}">\n')
    lines.append(origin_xml(pose, "    "))
    lines.append(f'    <parent link="{safe_name(parent)}" />\n')
    lines.append(f'    <child link="{safe_name(child)}" />\n')

    if jtype != "fixed":
        axis = get_text(joint, "axis/xyz", "0 0 1")
        lower = get_text(joint, "axis/limit/lower", "-3.14159")
        upper = get_text(joint, "axis/limit/upper", "3.14159")
        effort = get_text(joint, "axis/limit/effort", "35")
        velocity = get_text(joint, "axis/limit/velocity", "1.0")
        damping = get_text(joint, "axis/dynamics/damping", "0")
        friction = get_text(joint, "axis/dynamics/friction", "0")

        lines.append(f'    <axis xyz="{axis}" />\n')
        lines.append(
            f'    <limit lower="{lower}" upper="{upper}" '
            f'effort="{effort}" velocity="{velocity}" />\n'
        )
        lines.append(f'    <dynamics damping="{damping}" friction="{friction}" />\n')

    lines.append("  </joint>\n\n")

# Gazebo plugin은 MoveIt URDF에는 넣지 않음
lines.append("</robot>\n")

OUT_PATH.write_text("".join(lines), encoding="utf-8")

print("[OK] MoveIt URDF/Xacro generated")
print(f"[OK] input : {SDF_PATH}")
print(f"[OK] output: {OUT_PATH}")
