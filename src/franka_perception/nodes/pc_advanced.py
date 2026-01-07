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


def smart_plane_removal(pc, ransac_dist=0.005, plane_z_buffer=0.002):
    """
    智能桌面去除（RANSAC平面分割）：
    1. RANSAC平面拟合 - 找到桌面
    2. 移除平面inliers
    3. 只保留桌面上方 [2mm, 80mm] 的点
    
    Args:
        pc: Open3D点云
        ransac_dist: RANSAC距离阈值（默认5mm）
        plane_z_buffer: 离平面的最小距离（默认2mm）
    
    Returns:
        filtered_pc: 过滤后的点云
        plane_model: 桌面平面模型 [a, b, c, d]
        plane_inliers: 平面inlier的索引
    """
    points = np.asarray(pc.points)
    
    if len(points) < 10:
        return pc, None, []
    
    # Step 1: RANSAC平面拟合
    try:
        plane_model, inliers = pc.segment_plane(
            distance_threshold=ransac_dist,
            ransac_n=3,
            num_iterations=1000
        )
    except:
        return pc, None, []
    
    if len(inliers) < 5:
        return pc, None, []
    
    [a, b, c, d] = plane_model
    
    # Step 2: 计算所有点到平面的有符号距离
    plane_normal = np.array([a, b, c])
    plane_norm = np.linalg.norm(plane_normal)
    plane_normal_normalized = plane_normal / plane_norm
    
    # 有符号距离（点在平面上方为正）
    signed_distances = np.dot(points, plane_normal_normalized) + d / plane_norm
    
    # Step 3: 只保留 [plane_z_buffer, 0.08] 范围内的点（离桌面2-80mm）
    # cube高度是45mm，加buffer后最多80mm
    height_min = plane_z_buffer
    height_max = 0.08  # 80mm
    
    height_mask = (signed_distances >= height_min) & (signed_distances <= height_max)
    
    if np.sum(height_mask) == 0:
        return pc, None, []
    
    # Step 4: 获取过滤后的点云
    filtered_indices = np.where(height_mask)[0]
    filtered_pc = pc.select_by_index(filtered_indices.tolist())
    
    return filtered_pc, plane_model, inliers


def height_bandpass_filter(pc, plane_model, height_min=0.002, height_max=0.080):
    """
    高度带通滤波（在平面上方）：
    只保留 [height_min, height_max] 范围内的点
    默认：离桌面 2mm - 80mm（cube高45mm + 额外buffer）
    
    Args:
        pc: Open3D点云
        plane_model: 桌面平面模型 [a, b, c, d]
        height_min: 最小高度（米，默认2mm）
        height_max: 最大高度（米，默认80mm）
    
    Returns:
        filtered_pc: 过滤后的点云
    """
    if plane_model is None or len(np.asarray(pc.points)) == 0:
        return pc
    
    points = np.asarray(pc.points)
    [a, b, c, d] = plane_model
    
    # 计算点到平面的有符号距离
    plane_normal = np.array([a, b, c])
    plane_norm = np.linalg.norm(plane_normal)
    plane_normal_normalized = plane_normal / plane_norm
    
    signed_distances = np.dot(points, plane_normal_normalized) + d / plane_norm
    
    # 高度带通：[height_min, height_max]
    height_mask = (signed_distances >= height_min) & (signed_distances <= height_max)
    
    if np.sum(height_mask) == 0:
        return pc  # 没有点符合条件，返回原始点云
    
    return pc.select_by_index(np.where(height_mask)[0].tolist())


def validate_icp_result(icp_result, min_fitness=0.3, max_rmse=0.020):
    """
    验证ICP结果的质量（改进方案）
    
    改进点：
    - 综合使用 fitness + RMSE（两者都要通过）
    - 不盲目相信 fitness=1.0（但仍然需要满足最低门槛）
    - correspondence_set 作为辅助参考（可能不可用）
    
    Args:
        icp_result: ICP注册结果
        min_fitness: 最小fitness阈值（默认0.3）
        max_rmse: 最大RMSE阈值（默认2cm）
    
    Returns:
        is_valid: 是否有效
        quality_score: 0-1的质量分数
        reasons: 验证失败原因列表
    """
    
    # 提取关键指标
    fitness = icp_result.fitness
    rmse = icp_result.inlier_rmse
    
    # 尝试获取correspondence信息（可能不可用）
    correspondence_set = getattr(icp_result, 'correspondence_set', None)
    correspondence_count = len(correspondence_set) if correspondence_set is not None else 0
    
    # 验证规则（综合fitness和RMSE）
    reasons = []
    
    if fitness < min_fitness:
        reasons.append(f"low_fitness({fitness:.3f}<{min_fitness})")
    
    if rmse > max_rmse:
        reasons.append(f"high_rmse({rmse:.4f}>{max_rmse})")
    
    is_valid = len(reasons) == 0
    
    # 计算质量分数（综合指标）
    # fitness 贡献：0-1（normalize到0.8）
    fitness_score = min(fitness / 0.8, 1.0)
    # rmse 贡献：高RMSE则低分
    rmse_score = max(1.0 - (rmse / (max_rmse * 1.5)), 0.0)
    
    quality_score = (fitness_score + rmse_score) / 2.0
    
    return is_valid, quality_score, reasons


# 测试函数
def test_icp_validation():
    """测试ICP验证函数"""
    print("ICP 质量验证测试（fitness + RMSE）")
    print("=" * 60)
    
    # 模拟ICP结果
    class FakeResult:
        def __init__(self, fitness, rmse):
            self.fitness = fitness
            self.inlier_rmse = rmse
            self.correspondence_set = None
    
    test_cases = [
        (1.0, 0.005, "完美配准"),
        (0.8, 0.010, "好的配准"),
        (0.5, 0.015, "中等配准"),
        (0.3, 0.020, "临界配准"),
        (0.1, 0.030, "差的配准"),
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
