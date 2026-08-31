import xml.etree.ElementTree as ET
from pathlib import Path

urdf_path = Path(
    "osl_v2_0_assembly/urdf/osl_v2_0_assembly.urdf"
)

root = ET.parse(urdf_path).getroot()

print(f"Robot: {root.attrib.get('name')}")
print("\nMOVABLE JOINTS:\n")

for joint in root.findall("joint"):
    joint_type = joint.attrib.get("type")

    if joint_type in ("revolute", "continuous", "prismatic"):
        name = joint.attrib.get("name")
        parent = joint.find("parent").attrib.get("link")
        child = joint.find("child").attrib.get("link")

        axis_element = joint.find("axis")
        axis = axis_element.attrib.get("xyz") if axis_element is not None else "N/A"

        print(f"Name   : {name}")
        print(f"Type   : {joint_type}")
        print(f"Parent : {parent}")
        print(f"Child  : {child}")
        print(f"Axis   : {axis}")
        print("-" * 60)