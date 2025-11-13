#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

class OccupancyGridMap:

    def __init__(self, resolution=0.1, width_meters=20, height_meters=20):
        # 地图参数
        self.resolution = resolution
        self.width_meters = width_meters
        self.height_meters = height_meters
        self.width_pixels = int(width_meters / resolution)
        self.height_pixels = int(height_meters / resolution)
        
        # 地图数据: 0=未知, 1=空闲, -1=占用
        self.grid = np.zeros((self.height_pixels, self.width_pixels), dtype=np.int8)

        # 地图原点在中心
        self.origin_x = -self.width_meters / 2.0
        self.origin_z = -self.height_meters / 2.0  # 更改为z轴

        # 用于记录机器人路径
        self.robot_path = []
        # 用于记录上一个添加到路径中的点，避免路径点过于密集
        self.last_path_point = None

    # ==================== 坐标转换方法 ====================
    def meters_to_pixels(self, x_m, z_m):
        px = int((x_m - self.origin_x) / self.resolution)
        pz = int((z_m - self.origin_z) / self.resolution)  # 更改为z轴
        return px, pz

    def pixels_to_meters(self, px, pz):
        x_m = px * self.resolution + self.origin_x + self.resolution / 2.0
        z_m = pz * self.resolution + self.origin_z + self.resolution / 2.0  # 更改为z轴
        return x_m, z_m

    # ==================== 地图更新方法 ====================
    def update_cell(self, x_m, z_m, value):  # 更改为z轴
        px, pz = self.meters_to_pixels(x_m, z_m)
        if 0 <= px < self.width_pixels and 0 <= pz < self.height_pixels:
            self.grid[pz, px] = value  # 简单覆盖

    def update_free_space(self, robot_x_m, robot_z_m, scan_points_m):  # 更改为z轴
        robot_px, robot_pz = self.meters_to_pixels(robot_x_m, robot_z_m)
        for point_x_m, point_z_m in scan_points_m:  # 更改为z轴
            point_px, point_pz = self.meters_to_pixels(point_x_m, point_z_m)
            # 使用 numpy 的 linspace 简单实现
            num_steps = max(abs(point_px - robot_px), abs(point_pz - robot_pz))
            if num_steps > 0:
                x_steps = np.linspace(robot_px, point_px, num_steps+1, dtype=int)
                z_steps = np.linspace(robot_pz, point_pz, num_steps+1, dtype=int)  # 更改为z轴
                for x_s, z_s in zip(x_steps[:-1], z_steps[:-1]):  # 更改为z轴
                    if 0 <= x_s < self.width_pixels and 0 <= z_s < self.height_pixels:
                        self.grid[z_s, x_s] = 1  # 标记为自由空间

    def update_occupied_cells(self, scan_points_m, occupied_value=-1):
        for x_m, z_m in scan_points_m:  # 更改为z轴
            self.update_cell(x_m, z_m, occupied_value)
    
    # ==================== 路径记录方法 ====================
    def add_to_path(self, x_m, z_m):  # 更改为z轴
        # 检查是否应该添加新的路径点（避免点过于密集）
        if self.last_path_point is None:
            self.robot_path.append((x_m, z_m))  # 更改为z轴
            self.last_path_point = (x_m, z_m)
        else:
            # 计算与上一个点的距离
            last_x, last_z = self.last_path_point  # 更改为z轴
            distance = math.sqrt((x_m - last_x)**2 + (z_m - last_z)**2)  # 更改为z轴
            # 只有当距离超过一个网格分辨率时才添加新点
            if distance >= self.resolution:
                self.robot_path.append((x_m, z_m))  # 更改为z轴
                self.last_path_point = (x_m, z_m)

    # ==================== 显示和保存方法 ====================
    def draw_on_display(self, display):
        display_width = display.getWidth()
        display_height = display.getHeight()
        # 清空显示
        display.setColor(0x000000)  # 黑色
        display.fillRectangle(0, 0, display_width, display_height)

        # 计算缩放比例，将地图缩放到 Display 尺寸
        scale_x = display_width / self.width_pixels
        scale_z = display_height / self.height_pixels  # 更改为z轴
        scale = min(scale_x, scale_z)  # 保持比例

        # 绘制地图
        for z in range(self.height_pixels):  # 更改为z轴
            for x in range(self.width_pixels):
                cell_value = self.grid[z, x]  # 更改为z轴
                color = 0x808080  # 默认灰色 (未知)
                if cell_value == 1:  # 空闲
                    color = 0xFFFFFF  # 白色
                elif cell_value == -1:  # 占用
                    color = 0x000000  # 黑色

                # 计算在 Display 上的绘制坐标和尺寸
                disp_x = int(x * scale)
                disp_z = int(z * scale)  # 更改为z轴
                disp_w = max(1, int(scale))  # 确保至少绘制一个像素
                disp_h = max(1, int(scale))

                display.setColor(color)
                display.fillRectangle(disp_x, disp_z, disp_w, disp_h)  # 更改为z轴
        
        # 绘制机器人路径
        path_color = 0xFF0000  # 红色
        display.setColor(path_color)
        if len(self.robot_path) > 1:
            # 绘制路径线段
            for i in range(len(self.robot_path) - 1):
                x1_m, z1_m = self.robot_path[i]  # 更改为z轴
                x2_m, z2_m = self.robot_path[i+1]  # 更改为z轴
                px1, pz1 = self.meters_to_pixels(x1_m, z1_m)  # 更改为z轴
                px2, pz2 = self.meters_to_pixels(x2_m, z2_m)  # 更改为z轴
                disp_x1 = int(px1 * scale)
                disp_z1 = int(pz1 * scale)  # 更改为z轴
                disp_x2 = int(px2 * scale)
                disp_z2 = int(pz2 * scale)  # 更改为z轴
                display.drawLine(disp_x1, disp_z1, disp_x2, disp_z2)  # 更改为z轴

    def save_to_csv(self, filename):
        np.savetxt(filename, self.grid, delimiter=',', fmt='%d')
        print(f"Map saved to {filename}")

    # ==================== 信息获取方法 ====================
    def get_map_data(self):
        return self.grid.copy()
        
    def get_path_data(self):
        return self.robot_path.copy()

    def get_map_info(self):
        return self.resolution, self.width_meters, self.height_meters, self.origin_x, self.origin_z  # 更改为z轴