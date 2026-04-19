#!/usr/bin/env python3

"""
增强版URDF到SDF转换器
支持Gazebo插件和传感器配置
"""

import os
import sys
from xml.etree import ElementTree as ET

def enhanced_urdf_to_sdf(urdf_path, sdf_path):
    """
    将增强版URDF转换为完整的SDF格式
    """
    # 读取URDF文件
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # 创建SDF根元素
    sdf_root = ET.Element('sdf', {'version': '1.8'})
    
    # 创建世界元素
    world = ET.SubElement(sdf_root, 'world', {'name': 'default'})
    
    # 添加物理引擎配置
    physics = ET.SubElement(world, 'physics', {'name': 'default_physics', 'default': 'true', 'type': 'ode'})
    max_step_size = ET.SubElement(physics, 'max_step_size')
    max_step_size.text = '0.001'
    real_time_factor = ET.SubElement(physics, 'real_time_factor')
    real_time_factor.text = '1.0'
    real_time_update_rate = ET.SubElement(physics, 'real_time_update_rate')
    real_time_update_rate.text = '1000'
    
    # 添加地面
    ground = ET.SubElement(world, 'model', {'name': 'ground_plane'})
    static_elem = ET.SubElement(ground, 'static')
    static_elem.text = 'true'
    ground_link = ET.SubElement(ground, 'link', {'name': 'link'})
    
    # 地面碰撞
    ground_collision = ET.SubElement(ground_link, 'collision', {'name': 'collision'})
    ground_geom = ET.SubElement(ground_collision, 'geometry')
    ground_plane = ET.SubElement(ground_geom, 'plane')
    ground_normal = ET.SubElement(ground_plane, 'normal')
    ground_normal.text = '0 0 1'
    
    # 地面视觉
    ground_visual = ET.SubElement(ground_link, 'visual', {'name': 'visual'})
    ground_vis_geom = ET.SubElement(ground_visual, 'geometry')
    ground_vis_plane = ET.SubElement(ground_vis_geom, 'plane')
    ground_vis_normal = ET.SubElement(ground_vis_plane, 'normal')
    ground_vis_normal.text = '0 0 1'
    ground_size = ET.SubElement(ground_vis_plane, 'size')
    ground_size.text = '100 100'
    
    # 主机器人模型
    model = ET.SubElement(world, 'model', {'name': 'zhongxi_robot'})
    
    # 处理每个link
    for link in root.findall('link'):
        sdf_link = ET.SubElement(model, 'link', {'name': link.get('name')})
        
        # 惯性信息
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
        
        # 视觉信息
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
                    
                    # 添加材质
                    material = visual.find('material')
                    if material is not None:
                        sdf_material = ET.SubElement(sdf_visual, 'material')
                        color = material.find('color')
                        if color is not None:
                            ambient = ET.SubElement(sdf_material, 'ambient')
                            ambient.text = color.get('rgba', '0.8 0.8 0.8 1')
                            diffuse = ET.SubElement(sdf_material, 'diffuse')
                            diffuse.text = color.get('rgba', '0.8 0.8 0.8 1')
        
        # 碰撞信息
        collision = link.find('collision')
        if collision is not None:
            sdf_collision = ET.SubElement(sdf_link, 'collision', {'name': 'collision'})
            origin = collision.find('origin')
            if origin is not None:
                pose = ET.SubElement(sdf_collision, 'pose')
                xyz = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                pose.text = f"{xyz} {rpy}"
            
            geometry = collision.find('geometry')
            if geometry is not None:
                sdf_geometry = ET.SubElement(sdf_collision, 'geometry')
                mesh = geometry.find('mesh')
                if mesh is not None:
                    sdf_mesh = ET.SubElement(sdf_geometry, 'mesh')
                    uri = ET.SubElement(sdf_mesh, 'uri')
                    filename = mesh.get('filename', '')
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
    
    # 添加Gazebo插件（传感器等）
    # 激光雷达传感器
    laser_link_found = any(link.get('name') == 'laser_link' for link in root.findall('link'))
    if laser_link_found:
        laser_plugin = ET.SubElement(world, 'plugin', {
            'name': 'laser_sensor',
            'filename': 'libignition-gazebo-sensors-system.so'
        })
        laser_topic = ET.SubElement(laser_plugin, 'topic')
        laser_topic.text = '/world/default/model/zhongxi_robot/link/laser_link/sensor/laser_sensor/scan'
    
    # 相机传感器
    camera_link_found = any(link.get('name') == 'camera_link' for link in root.findall('link'))
    if camera_link_found:
        camera_plugin = ET.SubElement(world, 'plugin', {
            'name': 'camera_sensor',
            'filename': 'libignition-gazebo-sensors-system.so'
        })
        camera_topic = ET.SubElement(camera_plugin, 'topic')
        camera_topic.text = '/world/default/model/zhongxi_robot/link/camera_link/sensor/camera_sensor/image'
    
    # 保存SDF文件
    sdf_tree = ET.ElementTree(sdf_root)
    sdf_tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
    
    print(f"增强版SDF文件已生成: {sdf_path}")

if __name__ == '__main__':
    urdf_file = '/home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description_enhanced.urdf'
    sdf_file = '/home/chuil/Desktop/zhongxi/src/zhongxi_description/urdf/zhongxi_description_enhanced.sdf'
    
    if os.path.exists(urdf_file):
        enhanced_urdf_to_sdf(urdf_file, sdf_file)
    else:
        print(f"错误: 找不到增强版URDF文件 {urdf_file}")
        sys.exit(1)