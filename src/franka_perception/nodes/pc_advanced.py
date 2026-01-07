#!/usr/bin/env python3
"""
改进的点云处理：智能化Z轴处理和ICP质量检查

关键改进：
1. ICP fitness gate - 不再盲目相信fitness=1.0
2. Z轴高度带通滤波 - 移除桌面和上表面点
3. 更严格的点云预处理
"""

import numpy as np
import open3d as o3d


def smart_plane_removal(pc, ransac_dist, plane_z_buffer=0.01):
    """
    智能桌面去除：
    1. 拟合桌面平面
    2. 提取平面上方的点
    3. 应用高度带通滤波
    
    Args:
        pc: Open3D点云
        ransac_dist: RANSAC距离阈值
        plane_z_buffer: 桌面上方保留的距离（米）
    
    Returns:
        filtered_pc: 过滤后的点云
        plane_model: 桌面平面模型 [a, b, c, d]
    """
    points = np.asarray(pc.points)
    
    if len(points) < 10:
        return pc, None
    
    # Step 1: 拟合桌面平面（RANSAC）
    plane_model, inliers = pc.segment_plane(
        distance_threshold=ransac_dist,
        ransac_n=3,
        num_iterations=1000
    )
    
    [a, b, c, d] = plane_model
    
    # Step 2: 计算点到平面的距离
    # 平面方程: ax + by + cz + d = 0
    # 距离 = |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
    plane_normal = np.array([a, b, c])
    plane_normal_normalized = plane_normal / np.linalg.norm(plane_normal)
    
    distances_to_plane = np.abs(np.dot(points, plane_normal_normalized) + d)
    
    # Step 3: 移除桌面点（距离 < plane_z_buffer）
    table_mask = distances_to_plane > plane_z_buffer
    
    # Step 4: 获取桌面上方的点
    above_plane_pc = pc.select_by_index(np.where(table_mask)[0])
    
    return above_plane_pc, plane_model


def height_bandpass_filter(pc, plane_model, cube_height=0.045, buffer=0.01):
    """
    高度带通滤波：
    只保留 plane_上方 到 plane_上方+height 范围内的点
    
    Args:
        pc: Open3D点云
        plane_model: 桌面平面模型 [a, b, c, d]
        cube_height: cube高度（0.045m）
        buffer: 额外缓冲（上下各加一点）
    
    Returns:
        filtered_pc: 过滤后的点云
    """
    if plane_model is None or len(np.asarray(pc.points)) == 0:
        return pc
    
    points = np.asarray(pc.points)
    [a, b, c, d] = plane_model
    
    # 计算点到平面的距离
    plane_normal = np.array([a, b, c])
    plane_normal_normalized = plane_normal / np.linalg.norm(plane_normal)
    
    distances_to_plane = np.dot(points, plane_normal_normalized) + d / np.linalg.norm(plane_normal)
    
    # 高度带通：[buffer, cube_height + buffer]
    height_min = buffer
    height_max = cube_height + buffer
    
    height_mask = (distances_to_plane >= height_min) & (distances_to_plane <= height_max)
    
    if np.sum(height_mask) == 0:
        return pc  # 没有点符合条件，返回原始点云
    
    return pc.select_by_index(np.where(height_mask)[0])


def validate_icp_result(icp_result, min_fitness=0.5, max_rmse=0.02):
    """
    验证ICP结果的质量
    
    Args:
        icp_result: ICP注册结果
        min_fitness: 最小fitness阈值
        max_rmse: 最大RMSE阈值
    
    Returns:
        is_valid: 是否有效
        quality_score: 0-1的质量分数
        reason: 验证失败原因（如果有）
    """
    fitness = icp_result.fitness
    rmse = icp_result.inlier_rmse
    
    # 计算质量分数（综合指标）
    fitness_score = min(fitness / 0.8, 1.0)  # normalize to 0.8
    rmse_score = max(1.0 - (rmse / max_rmse), 0.0) if rmse > 0 else 1.0
    
    quality_score = (fitness_score + rmse_score) / 2.0
    
    # 验证规则
    reasons = []
    
    if fitness < min_fitness:
        reasons.append(f"low_fitness({fitness:.3f}<{min_fitness})")
    
    if rmse > max_rmse:
        reasons.append(f"high_rmse({rmse:.4f}>{max_rmse})")
    
    is_valid = len(reasons) == 0
    
    return is_valid, quality_score, reasons


# 测试函数
def test_icp_validation():
    """测试ICP验证函数"""
    print("ICP 质量验证测试")
    print("=" * 60)
    
    # 模拟ICP结果
    class FakeResult:
        def __init__(self, fitness, rmse):
            self.fitness = fitness
            self.inlier_rmse = rmse
    
    test_cases = [
        (1.0, 0.001, "完美配准"),
        (0.8, 0.015, "好的配准"),
        (0.5, 0.025, "临界配准"),
        (0.3, 0.050, "差的配准"),
    ]
    
    for fitness, rmse, label in test_cases:
        result = FakeResult(fitness, rmse)
        is_valid, score, reasons = validate_icp_result(result)
        
        status = "✓" if is_valid else "✗"
        print(f"\n{status} {label}")
        print(f"  Fitness: {fitness:.3f}, RMSE: {rmse:.4f}")
        print(f"  质量分数: {score:.3f}")
        if reasons:
            print(f"  失败原因: {', '.join(reasons)}")


if __name__ == '__main__':
    test_icp_validation()
