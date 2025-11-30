
import xml.etree.ElementTree as ET
import math

def get_pose(pose_str):
    parts = [float(x) for x in pose_str.split()]
    return parts

def format_origin(pose, parent_pose=[0,0,0,0,0,0]):
    # Simple approximation for translation: child_pos - parent_pos
    # This works because parent rotation is 0 for base_link
    # If parent rotation was not 0, we'd need matrix math.
    # Here base_link rpy is 0 0 0.
    
    x = pose[0] - parent_pose[0]
    y = pose[1] - parent_pose[1]
    z = pose[2] - parent_pose[2]
    r, p, yaw = pose[3], pose[4], pose[5]
    return f'xyz="{x} {y} {z}" rpy="{r} {p} {yaw}"'

def format_inertial(inertial_node):
    mass = inertial_node.find('mass').text
    inertia = inertial_node.find('inertia')
    ixx = inertia.find('ixx').text
    ixy = inertia.find('ixy').text
    ixz = inertia.find('ixz').text
    iyy = inertia.find('iyy').text
    iyz = inertia.find('iyz').text
    izz = inertia.find('izz').text
    pose = inertial_node.find('pose')
    origin_str = ""
    if pose is not None:
        p = get_pose(pose.text)
        origin_str = f'<origin xyz="{p[0]} {p[1]} {p[2]}" rpy="{p[3]} {p[4]} {p[5]}"/>'
    else:
        origin_str = '<origin xyz="0 0 0" rpy="0 0 0"/>'
        
    return f"""<inertial>
      {origin_str}
      <mass value="{mass}"/>
      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" iyy="{iyy}" iyz="{iyz}" izz="{izz}"/>
    </inertial>"""

def format_geometry(geometry_node):
    if geometry_node.find('box') is not None:
        size = geometry_node.find('box/size').text
        return f'<box size="{size}"/>'
    elif geometry_node.find('cylinder') is not None:
        radius = geometry_node.find('cylinder/radius').text
        length = geometry_node.find('cylinder/length').text
        return f'<cylinder radius="{radius}" length="{length}"/>'
    elif geometry_node.find('mesh') is not None:
        uri = geometry_node.find('mesh/uri').text
        scale = geometry_node.find('mesh/scale')
        scale_str = f'scale="{scale.text}"' if scale is not None else 'scale="1 1 1"'
        # Fix URI
        new_uri = uri.replace("file://custom_hexa", "package://hexacopter_description/custom_hexa")
        return f'<mesh filename="{new_uri}" {scale_str}/>'
    return ""

def main():
    sdf_path = '/home/lahiru/newlocalrepo/rl-aerial-manipulator/hexacopter_description/custom_hexa/model.sdf'
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find('model')
    
    urdf_content = ['<?xml version="1.0" ?>', '<robot name="custom_hexa">']
    
    # Base Link Pose
    base_link = model.find(".//link[@name='base_link']")
    base_pose_str = base_link.find('pose').text
    base_pose = get_pose(base_pose_str)
    
    # Process Links
    for link in model.findall('link'):
        name = link.get('name')
        urdf_content.append(f'  <link name="{name}">')
        
        # Inertial
        inertial = link.find('inertial')
        if inertial is not None:
            urdf_content.append("    " + format_inertial(inertial))
            
        # Visuals
        for visual in link.findall('visual'):
            v_name = visual.get('name')
            v_pose = visual.find('pose')
            v_origin = ""
            if v_pose is not None:
                p = get_pose(v_pose.text)
                v_origin = f'<origin xyz="{p[0]} {p[1]} {p[2]}" rpy="{p[3]} {p[4]} {p[5]}"/>'
            else:
                v_origin = '<origin xyz="0 0 0" rpy="0 0 0"/>'
            
            geometry = format_geometry(visual.find('geometry'))
            
            # Material (Simple color extraction)
            material_str = ""
            mat = visual.find('material')
            if mat is not None:
                diffuse = mat.find('diffuse')
                if diffuse is not None:
                    rgba = diffuse.text
                    material_str = f'<material name="{v_name}_mat"><color rgba="{rgba}"/></material>'
            
            urdf_content.append(f"""    <visual>
      {v_origin}
      <geometry>
        {geometry}
      </geometry>
      {material_str}
    </visual>""")

        # Collisions
        for collision in link.findall('collision'):
            c_pose = collision.find('pose')
            c_origin = ""
            if c_pose is not None:
                p = get_pose(c_pose.text)
                c_origin = f'<origin xyz="{p[0]} {p[1]} {p[2]}" rpy="{p[3]} {p[4]} {p[5]}"/>'
            else:
                c_origin = '<origin xyz="0 0 0" rpy="0 0 0"/>'
                
            geometry = format_geometry(collision.find('geometry'))
            
            urdf_content.append(f"""    <collision>
      {c_origin}
      <geometry>
        {geometry}
      </geometry>
    </collision>""")
            
        urdf_content.append('  </link>')

    # Process Joints
    # In SDF, joints are defined with parent and child.
    # We need to calculate the relative transform.
    # We assume base_link is the parent for most, or we follow the chain.
    # The SDF provided has all joints connected to base_link.
    
    for joint in model.findall('joint'):
        name = joint.get('name')
        type_ = joint.get('type')
        parent = joint.find('parent').text
        child = joint.find('child').text
        
        # Find child link pose
        child_link = model.find(f".//link[@name='{child}']")
        child_pose_str = child_link.find('pose').text
        child_pose = get_pose(child_pose_str)
        
        # Find parent link pose (should be base_link for all in this file)
        parent_link = model.find(f".//link[@name='{parent}']")
        if parent_link is not None:
            parent_pose_str = parent_link.find('pose').text
            parent_pose = get_pose(parent_pose_str)
        else:
            # If parent is world, skip or handle? SDF has commented out fixed to world.
            # Assuming base_link is root, so we only care about joints where parent is in the model.
            continue

        origin_str = format_origin(child_pose, parent_pose)
        
        urdf_type = "fixed"
        if type_ == "revolute":
            urdf_type = "continuous" # Assuming continuous for rotors
            
        axis_str = ""
        if joint.find('axis') is not None:
            xyz = joint.find('axis/xyz').text
            axis_str = f'<axis xyz="{xyz}"/>'
            
        urdf_content.append(f"""  <joint name="{name}" type="{urdf_type}">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin {origin_str}/>
    {axis_str}
  </joint>""")

    urdf_content.append('</robot>')
    
    with open('/home/lahiru/newlocalrepo/rl-aerial-manipulator/hexacopter_description/custom_hexa/model.urdf', 'w') as f:
        f.write('\n'.join(urdf_content))

if __name__ == "__main__":
    main()
