from pathlib import Path
import re

xacro = Path("src/arm_only_description/urdf/arm_only_gripper_moveit.urdf.xacro")
text = xacro.read_text()

links = [
    "ARM_BASE_LINK",
    "LINK1",
    "LINK2",
    "LINK3",
    "LINK4",
    "LINK5",
    "LINK6",
    "gripper_base",
    "drive_gear",
    "LEFT_PLATE",
    "RIGHT_PLATE",
]

for link in links:
    m = re.search(rf'<link\s+name="{re.escape(link)}">[\s\S]*?</link>', text)
    if not m:
        continue

    block = m.group(0)
    collisions = list(re.finditer(r'<collision\s+name="([^"]+)"[\s\S]*?</collision>', block))

    print(f"\n===== {link} =====")
    if not collisions:
        print("  collision 없음")
        continue

    for c in collisions:
        col = c.group(0)
        name = c.group(1)

        origin = re.search(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>', col)
        box = re.search(r'<box\s+size="([^"]+)"\s*/>', col)
        cyl = re.search(r'<cylinder\s+radius="([^"]+)"\s+length="([^"]+)"\s*/>', col)

        print(f"  {name}")

        if origin:
            print(f"    xyz = {origin.group(1)}")
            print(f"    rpy = {origin.group(2)}")
        else:
            print("    origin 없음")

        if box:
            print(f"    box size = {box.group(1)}")
        elif cyl:
            print(f"    cylinder radius = {cyl.group(1)}, length = {cyl.group(2)}")
        else:
            print("    geometry = mesh or unknown")
