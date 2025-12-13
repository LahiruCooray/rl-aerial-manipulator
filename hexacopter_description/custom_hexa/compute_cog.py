#!/usr/bin/env python3
"""
Compute Center of Gravity (CoG) for the hexacopter model.
Parses model.sdf and calculates mass-weighted CoM position.
"""

import xml.etree.ElementTree as ET
import math

def compute_cog(sdf_file='model.sdf'):
    """Calculate CoG from SDF file."""
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    
    total_mass = 0.0
    total_mx = 0.0
    total_my = 0.0
    total_mz = 0.0
    
    for link in root.iter('link'):
        name = link.get('name')
        
        # Get link pose
        pose_elem = link.find('pose')
        if pose_elem is None:
            continue
        
        pose = pose_elem.text.strip().split()
        x_link, y_link, z_link = float(pose[0]), float(pose[1]), float(pose[2])
        
        # Get orientation (yaw only for 2D rotation)
        try:
            yaw = float(pose[5])
        except:
            yaw = 0.0
        
        # Get inertial properties
        for inertial in link.iter('inertial'):
            mass_elem = inertial.find('mass')
            if mass_elem is None:
                continue
            
            mass = float(mass_elem.text.strip())
            
            # Get inertial offset (local frame)
            inertial_pose_elem = inertial.find('pose')
            if inertial_pose_elem is not None:
                inertial_pose = inertial_pose_elem.text.strip().split()
                x_i_local = float(inertial_pose[0])
                y_i_local = float(inertial_pose[1])
                z_i_local = float(inertial_pose[2])
            else:
                x_i_local = y_i_local = z_i_local = 0.0
            
            # Transform inertial offset to world frame
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            x_i_world = x_i_local * cos_yaw - y_i_local * sin_yaw
            y_i_world = x_i_local * sin_yaw + y_i_local * cos_yaw
            z_i_world = z_i_local
            
            # Total CoM position in world frame
            x_total = x_link + x_i_world
            y_total = y_link + y_i_world
            z_total = z_link + z_i_world
            
            # Accumulate mass-weighted positions
            total_mass += mass
            total_mx += mass * x_total
            total_my += mass * y_total
            total_mz += mass * z_total
    
    # Calculate CoG
    cog_x = total_mx / total_mass
    cog_y = total_my / total_mass
    cog_z = total_mz / total_mass
    
    return total_mass, cog_x, cog_y, cog_z


if __name__ == '__main__':
    mass, x, y, z = compute_cog()
    
    print("="*60)
    print("HEXACOPTER CENTER OF GRAVITY")
    print("="*60)
    print(f"Total Mass:  {mass:.4f} kg")
    print(f"CoG Position:")
    print(f"  X: {x:>10.6f} m  ({x*1000:>8.4f} mm)")
    print(f"  Y: {y:>10.6f} m  ({y*1000:>8.4f} mm)")
    print(f"  Z: {z:>10.6f} m  ({z*1000:>8.4f} mm)")
    print("="*60)
    
    # Check symmetry
    if abs(x) < 0.001 and abs(y) < 0.001:
        print("✅ Model is symmetric in X-Y plane")
    else:
        print("⚠️  Model has lateral CoG offset:")
        if abs(x) > 0.001:
            print(f"   X-bias: {x*1000:.2f} mm")
        if abs(y) > 0.001:
            print(f"   Y-bias: {y*1000:.2f} mm")
