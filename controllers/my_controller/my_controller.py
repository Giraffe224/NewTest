#!/usr/bin/env python3

import csv
from controller import Robot
import sys
import numpy as np
import sys
import numpy as np

# 尝试导入自定义的地图系统模块
try:
    import occupancy_grid_map
    import map_builder
    MAP_SYSTEM_AVAILABLE = True
except ImportError:
    MAP_SYSTEM_AVAILABLE = False
    print("警告: 无法导入地图系统模块，地图功能将不可用")

print("我是神人")

# 仿真时间步长（毫秒）
TIME_STEP = 32

# =============== 机器人硬件初始化 ===============
# 创建机器人实例
robot = Robot()

# 轮子电机列表
wheels = []

# 距离传感器列表
distance_sensors = []

# =============== GPS设备初始化 ===============
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
# =============================================

# =============== 惯性测量单元初始化 ===============
inertial_unit = None
try:
    inertial_unit = robot.getDevice('inertial unit')
    inertial_unit.enable(TIME_STEP)
    print("惯性测量单元已启用")
except:
    print("未找到惯性测量单元，将使用GPS数据估算朝向")
# =============================================

# =============== 激光雷达初始化 ===============
lidar = robot.getDevice('LDS-01')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
NUM_LIDAR_POINTS = lidar.getHorizontalResolution()
# =============================================

# =============== Display显示设备初始化 ===============
# 尝试获取并初始化Display设备用于绘制地图
display = None
try:
    display = robot.getDevice("display")
    display_width = display.getWidth()
    display_height = display.getHeight()
    # 填充黑色背景
    display.fill(0x000000)
    print(f"Display设备已初始化: {display_width}x{display_height}")
except:
    print("未找到Display设备,将不显示实时地图")
# =============================================

# =============== CSV数据记录初始化 ===============
csv_file = open('lds_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
# 表头：timestamp + 激光雷达距离数据 + GPS坐标
header = ['timestamp'] + [f'dist_{i}' for i in range(NUM_LIDAR_POINTS)] + ['gps_x', 'gps_y', 'gps_z']
csv_writer.writerow(header)
frame_count = 0
data_save_interval = 50  # 每多少帧保存一次数据到CSV文件
# =================================================

# =============== 地图系统初始化 ===============
occupancy_map = None
map_processor = None
map_system_ready = False

if MAP_SYSTEM_AVAILABLE and display is not None:
    try:
        # 初始化地图 (分辨率为0.05米，地图大小为20x20米)
        occupancy_map = occupancy_grid_map.OccupancyGridMap(resolution=0.05, width_meters=20, height_meters=20)
        map_processor = map_builder.MapBuilder(occupancy_map)
        map_system_ready = True
        print("新地图系统初始化成功")
    except Exception as e:
        print(f"地图系统初始化失败: {e}")
# =============================================

# =============== 轮子电机初始化 ===============
wheel_names = ['left_motor', 'right_motor']
for i in range(2):
    wheels.append(robot.getDevice(wheel_names[i]))      # 获取左右两个电机
    wheels[i].setPosition(float('inf'))                 # 设为速度模式（无限旋转）
    wheels[i].setVelocity(0.0)                          # 初始速度为0
# =============================================

# =============== 距离传感器初始化 ===============
distance_sensor_names = ['ds_right', 'ds_left']
for name in distance_sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    distance_sensors.append(sensor)
# =============================================

# =============== 避障控制参数 ===============
# 避障计数器和避障类型
obstacle_avoidance_counter = 0
obstacle_avoidance_type = 0     # 0:无避障, 1:右避障, 2:左避障, 3:后退避障
obstacle_threshold = 1000.0     # 障碍物检测阈值
# =============================================

# =============== 主控制循环 ===============
while robot.step(TIME_STEP) != -1:
    # 获取GPS数据
    gps_values = gps.getValues()
    
    # =============== 获取惯性测量单元数据 ===============
    # 尝试获取惯性测量单元数据以获得更准确的机器人朝向
    robot_roll = 0.0
    robot_pitch = 0.0
    robot_yaw = 0.0
    
    if inertial_unit is not None:
        try:
            # 获取惯性测量单元数据 [roll, pitch, yaw]
            imu_data = inertial_unit.getRollPitchYaw()
            robot_roll = imu_data[0]
            robot_pitch = imu_data[1]
            robot_yaw = imu_data[2]
        except:
            print("无法从惯性测量单元获取数据")
    # =============================================
    
    # ======== 采集雷达+GPS数据并更新地图 ========
    frame_count += 1
    if frame_count % data_save_interval == 0:
        # 获取激光雷达数据
        lidar_ranges = lidar.getRangeImage()
        timestamp = robot.getTime()
        
        # 将数据写入CSV文件
        csv_writer.writerow([timestamp] + list(lidar_ranges) + gps_values)
        
        # 使用地图系统更新地图
        if map_system_ready:
            try:
                robot_x = gps_values[0]
                robot_z = gps_values[2]  # 使用Z坐标作为机器人的z轴位置（而不是Y坐标）
                
                # 添加当前位置到机器人路径 (x和z轴)
                occupancy_map.add_to_path(robot_x, robot_z)
                
                # 估算机器人朝向（如果无法从惯性测量单元获取）
                if inertial_unit is None:
                    robot_yaw = map_processor.get_robot_yaw_from_gps(gps_values)
                
                # 处理扫描数据并更新地图
                map_processor.process_scan_and_position(lidar_ranges, robot_x, robot_z, robot_yaw)
                
                # 在Display上绘制地图
                occupancy_map.draw_on_display(display)
            except Exception as e:
                print(f"地图更新错误: {e}")
    # =============================================
    
    # =============== 运动控制逻辑 ===============
    # 默认前进速度设置
    left_speed = 3.0
    right_speed = -3.0   # 注意：右轮为负值表示正向旋转（根据具体机器人结构而定）
    
    # 如果正在进行避障动作
    if obstacle_avoidance_counter > 0:
        # 减少避障计数器
        obstacle_avoidance_counter -= 1
        
        # 根据避障类型执行相应的动作
        if obstacle_avoidance_type == 1:  # 右侧有障碍，向左转
            left_speed = 5.0    # 左轮倒转
            right_speed = 5.0   # 右轮倒转（实现左转）
            print("向左转避障...")
        elif obstacle_avoidance_type == 2:  # 左侧有障碍，向右转
            left_speed = -5.0   # 左轮正转
            right_speed = -5.0  # 右轮正转（实现右转）
            print("向右转避障...")
        elif obstacle_avoidance_type == 3:  # 前方都有障碍，后退
            left_speed = -5.0   # 左轮倒转（后退）
            right_speed = 4.0   # 右轮正转（辅助后退转向）
            print("后退避障...")
            
        # 避障动作结束后重置避障类型
        if obstacle_avoidance_counter == 0:
            obstacle_avoidance_type = 0
            print("避障结束，恢复前行")
    else:
        # 正常前进模式
        # 读取传感器值
        right_sensor_value = distance_sensors[0].getValue()   # 右侧传感器
        left_sensor_value = distance_sensors[1].getValue()    # 左侧传感器
        
        # 打印传感器值用于调试
        print(f"右侧传感器: {right_sensor_value:.2f}, 左侧传感器: {left_sensor_value:.2f}")
        
        # 根据传感器读数决定避障动作
        if right_sensor_value < obstacle_threshold and left_sensor_value >= obstacle_threshold:
            # 只右侧有障碍，向左转
            obstacle_avoidance_type = 1
            obstacle_avoidance_counter = 30  # 执行指定时间步的避障动作
            print("右侧检测到障碍物，开始向左转避障")
        elif left_sensor_value < obstacle_threshold and right_sensor_value >= obstacle_threshold:
            # 只左侧有障碍，向右转
            obstacle_avoidance_type = 2
            obstacle_avoidance_counter = 30  # 执行指定时间步的避障动作
            print("左侧检测到障碍物，开始向右转避障")
        elif right_sensor_value < obstacle_threshold and left_sensor_value < obstacle_threshold:
            # 两侧都有障碍，后退并向右转
            obstacle_avoidance_type = 3
            obstacle_avoidance_counter = 30  # 执行指定时间步的避障动作
            print("两侧检测到障碍物，开始后退避障")
        
    # 设置车轮速度
    wheels[0].setVelocity(left_speed)
    wheels[1].setVelocity(right_speed)
    
    # 打印GPS坐标信息
    print(f"GPS坐标: x={gps_values[0]:.2f}, y={gps_values[1]:.2f}, z={gps_values[2]:.2f}")
# =============================================

# =============== 程序结束处理 ===============
csv_file.close()