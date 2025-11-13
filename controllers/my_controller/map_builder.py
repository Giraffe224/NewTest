#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

class MapBuilder:
    
    def __init__(self, occupancy_grid_map, max_range=3.5):
        # 地图实例和最大探测范围
        self.map = occupancy_grid_map
        self.max_range = max_range
        self.previous_gps_values = None
        
    # ==================== 数据处理方法 ====================
    def process_scan_and_position(self, laser_data, robot_x_m, robot_z_m, robot_yaw_rad):
        # 如果没有数据则跳过
        if not laser_data:
            return

        # 计算激光雷达的角度范围 (假设 RobotisLds01 是 360 度)
        num_points = len(laser_data)
        angle_increment = 2 * math.pi / num_points

        # 存储有效的扫描点
        scan_points_m = []
        for i, distance in enumerate(laser_data):
            # 忽略过近和过远的无效数据
            if 0.05 < distance < self.max_range:
                # 计算扫描点角度，加上机器人的朝向
                angle = -math.pi + i * angle_increment + robot_yaw_rad
                # 将极坐标转换为笛卡尔坐标，相对于机器人
                x_local = distance * math.cos(angle)
                z_local = distance * math.sin(angle)  # 使用z轴而不是y轴
                # 转换到世界坐标系
                x_world = x_local + robot_x_m
                z_world = z_local + robot_z_m  # 使用z轴而不是y轴
                scan_points_m.append((x_world, z_world))

        # 更新自由空间
        self.map.update_free_space(robot_x_m, robot_z_m, scan_points_m)

        # 更新障碍物点
        self.map.update_occupied_cells(scan_points_m)

    # ==================== 姿态估计方法 ====================
    def get_robot_yaw_from_gps(self, gps_values):
        # 如果没有历史数据，假设初始朝向为 0
        if self.previous_gps_values is None:
            self.previous_gps_values = gps_values
            return 0.0

        # 计算位置变化
        dx = gps_values[0] - self.previous_gps_values[0]
        dz = gps_values[2] - self.previous_gps_values[2]  # 使用z轴而不是y轴
        self.previous_gps_values = gps_values

        # 没有移动则返回0
        if dx == 0 and dz == 0:
            return 0.0

        # 计算偏航角
        yaw = math.atan2(dz, dx)  # 使用z轴而不是y轴
        return yaw