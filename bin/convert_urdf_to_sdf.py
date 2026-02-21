#!/usr/bin/env python3

"""
URDF 到 SDF 转换脚本
专门为 Ignition Gazebo 6.16.0 准备
"""

import os
import sys
from xml.etree import ElementTree as ET

def urdf_to_sdf(urdf_path, sdf_path):
    """
    将URDF文件转换为基本的SDF格式
    """
    # 读取URDF文件
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # 创建SDF根元素
    sdf_root = ET.Element('sdf', {'version': '1.8'})
    
    # 创建世界元素
    world = ET.SubElement(sdf_root, 'world', {'name': 'default'})
    
    # 添加物理引擎
    physics = ET.SubElement(world, 'physics', {'name': 'default_physics', 'default': 'true', 'type': 'ode'})
    max_step_size = ET.SubElement(physics, 'max_step_size')
    max_step_size.text = '0.001'
    real_time_factor = ET.SubElement(physics, 'real_time_factor')
    real_time_factor.text = '1.0'
    
    # 添加地面
    ground = ET.SubElement(world, 'model', {'name': 'ground_plane'})
    static_elem = ET.SubElement(ground, 'static')
    static_elem.text = 'true'
    link = ET.SubElement(ground, 'link', {'name': 'link'})
    collision = ET.SubElement(link, 'collision', {'name': 'collision'})
    geometry = ET.SubElement(collision, 'geometry')
    plane = ET.SubElement(geometry, 'plane')
    normal = ET.SubElement(plane, 'normal')
    normal.text = '0 0 1'
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    vis_geometry = ET.SubElement(visual, 'geometry')
    vis_plane = ET.SubElement(vis_geometry, 'plane')
    vis_normal = ET.SubElement(vis_plane, 'normal')
    vis_normal.text = '0 0 1'
    size = ET.SubElement(vis_plane, 'size')
    size.text = '100 100'
    
    # 复制机器人的link和joint信息
    model = ET.SubElement(world, 'model', {'name': 'zhongxi_robot'})
    
    # 处理每个link
    for link in root.findall('link'):
        sdf_link = ET.SubElement(model, 'link', {'name': link.get('name')})
        
        # 复制惯性信息
        inertial = link.find('inertial')
        if inertial is not None:
            sdf_inertial = ET.SubElement(sdf_link, 'inertial')
            
            mass_elem = inertial.find('mass')
            if mass_elem is not None:
                mass = ET.SubElement(sdf_inertial, 'mass')
                mass.text = mass_elem.get('value', '1.0')
            
            inertia = inertial.find('inertia')
            if inertia is not None:
                sdf_inertia = ET.SubElement(sdf_inertial, 'inertia')
                ixx = ET.SubElement(sdf_inertia, 'ixx')
                ixx.text = inertia.get('ixx', '1.0')
                iyy = ET.SubElement(sdf_inertia, 'iyy')
                iyy.text = inertia.get('iyy', '1.0')
                izz = ET.SubElement(sdf_inertia, 'izz')
                izz.text = inertia.get('izz', '1.0')
        
        # 复制视觉信息
        visual = link.find('visual')
        if visual is not None:
            sdf_visual = ET.SubElement(sdf_link, 'visual', {'name': 'visual'})
            origin = visual.find('origin')
            if origin is not None:
                pose = ET.SubElement(sdf_visual, 'pose')
                xyz = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                pose.text = f"{xyz} {rpy}"
            
            geometry = visual.find('geometry')
            if geometry is not None:
                sdf_geometry = ET.SubElement(sdf_visual, 'geometry')
                mesh = geometry.find('mesh')
                if mesh is not None:
                    sdf_mesh = ET.SubElement(sdf_geometry, 'mesh')
                    uri = ET.SubElement(sdf_mesh, 'uri')
                    filename = mesh.get('filename', '')
                    # 转换package://为file://
                    if filename.startswith('package://'):
                        filename = filename.replace('package://', 'file://')
                        filename = filename.replace('zhongxi_description', '/home/chuil/Desktop/zhongxi/src/zhongxi_description')
                    uri.text = filename
    
    # 处理关节
    for joint in root.findall('joint'):
        sdf_joint = ET.SubElement(model, 'joint', {
            'name': joint.get('name'),
            'type': joint.get('type')
        })
        
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            parent_elem = ET.SubElement(sdf_joint, 'parent')
            parent_elem.text = parent.get('link')
            child_elem = ET.SubElement(sdf_joint, 'child')
            child_elem.text = child.get('link')
        
        origin = joint.find('origin')
        if origin is not None:
            pose = ET.SubElement(sdf_joint, 'pose')
            xyz = origin.get('xyz', '0 0 0')
            rpy = origin.get('rpy', '0 0 0')
            pose.text = f"{xyz} {rpy}"
    
    # 保存SDF文件
    sdf_tree = ET.ElementTree(sdf_root)
    sdf_tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
    
    print(f"SDF文件已生成: {sdf_path}")

if __name__ == '__main__':
    urdf_file = '/home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description.urdf'
    sdf_file = '/home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description.sdf'
    
    if os.path.exists(urdf_file):
        urdf_to_sdf(urdf_file, sdf_file)
    else:
        print(f"错误: 找不到URDF文件 {urdf_file}")
        sys.exit(1)