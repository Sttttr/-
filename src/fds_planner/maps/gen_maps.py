#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 用于生成毕业设计 第6章 场景建模所需的 S1, S2, S3 地图

import os

def save_pgm(filename, grid):
    """保存为标准的 PGM 灰度图像格式"""
    h, w = len(grid), len(grid[0])
    with open(filename, 'wb') as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        for row in grid:
            f.write(bytearray(row))

def generate_yaml(name, resolution=0.05):
    """生成ROS map_server 需要的 yaml 配置文件"""
    yaml_content = f"""image: {name}.pgm
resolution: {resolution}
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
"""
    with open(f"{name}.yaml", "w") as f:
        f.write(yaml_content)

def draw_rect(grid, x_m, y_m, w_m, h_m, res=0.05):
    """在地图上画障碍物块矩形"""
    x0, y0 = int(x_m/res), int(y_m/res)
    w, h = int(w_m/res), int(h_m/res)
    for y in range(y0, y0+h):
        for x in range(x0, x0+w):
            if y < len(grid) and x < len(grid[0]):
                grid[y][x] = 0 # 0 代表黑色障碍物

def create_empty_map(width_m, height_m, res=0.05):
    """创建空地图并加上四周的墙壁"""
    w, h = int(width_m/res), int(height_m/res)
    grid = [[255 for _ in range(w)] for _ in range(h)] # 255是白色自由空间
    draw_rect(grid, 0, 0, width_m, 0.2, res) # 底墙
    draw_rect(grid, 0, height_m-0.2, width_m, 0.2, res) # 顶墙
    draw_rect(grid, 0, 0, 0.2, height_m, res) # 左墙
    draw_rect(grid, width_m-0.2, 0, 0.2, height_m, res) # 右墙
    return grid

# ==========================================
# 场景 S1：空旷区 (20x20m，4个静态障碍物)
# ==========================================
print("正在生成 S1 空旷区地图...")
s1 = create_empty_map(20, 20)
draw_rect(s1, 5, 5, 2, 2)
draw_rect(s1, 12, 4, 2, 3)
draw_rect(s1, 6, 14, 3, 2)
draw_rect(s1, 14, 14, 2, 2)
save_pgm("s1_mixed.pgm", s1)
generate_yaml("s1_mixed")

# ==========================================
# 场景 S2：狭窄走廊 (25x18m，中间一条 1.0m 窄道)
# ==========================================
print("正在生成 S2 狭窄走廊地图...")
s2 = create_empty_map(25, 18)
draw_rect(s2, 0, 7, 20, 3)   # 走廊上方墙壁
draw_rect(s2, 5, 11, 20, 3)  # 走廊下方墙壁
save_pgm("s2_corridor.pgm", s2)
generate_yaml("s2_corridor")

# ==========================================
# 场景 S3：U型障碍簇 (20x20m，制造局部极小值死锁陷阱)
# ==========================================
print("正在生成 S3 U型陷阱地图...")
s3 = create_empty_map(20, 20)
draw_rect(s3, 8, 8, 4, 1)   # U型底部
draw_rect(s3, 8, 12, 4, 1)  # U型顶部
draw_rect(s3, 11, 8, 1, 5)  # U型右侧封口 (开口向左)
save_pgm("s3_utrap.pgm", s3)
generate_yaml("s3_utrap")

print("所有地图生成完毕！")