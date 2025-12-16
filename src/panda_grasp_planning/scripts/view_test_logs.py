#!/usr/bin/env python3
"""
查看 Phase 2 自动化测试的日志文件
"""

import os
import sys
from datetime import datetime

def list_test_logs():
    """列出所有测试日志"""
    log_dir = os.path.expanduser("~/panda_grasp_planning/test_results")
    
    if not os.path.exists(log_dir):
        print(f"日志目录不存在: {log_dir}")
        return
    
    # 获取所有 auto_test_*.log 文件
    log_files = sorted([f for f in os.listdir(log_dir) if f.startswith("auto_test_") and f.endswith(".log")], reverse=True)
    
    if not log_files:
        print(f"在 {log_dir} 中没有找到任何测试日志")
        return
    
    print("=" * 70)
    print("PHASE 2 自动化测试日志")
    print("=" * 70)
    print()
    
    for i, log_file in enumerate(log_files, 1):
        file_path = os.path.join(log_dir, log_file)
        file_size = os.path.getsize(file_path)
        mod_time = os.path.getmtime(file_path)
        mod_time_str = datetime.fromtimestamp(mod_time).strftime("%Y-%m-%d %H:%M:%S")
        
        print(f"{i}. {log_file}")
        print(f"   时间: {mod_time_str}")
        print(f"   大小: {file_size} 字节")
        print()

def view_latest_log():
    """查看最新的日志文件"""
    log_dir = os.path.expanduser("~/panda_grasp_planning/test_results")
    
    if not os.path.exists(log_dir):
        print(f"日志目录不存在: {log_dir}")
        return
    
    log_files = sorted([f for f in os.listdir(log_dir) if f.startswith("auto_test_") and f.endswith(".log")], reverse=True)
    
    if not log_files:
        print(f"在 {log_dir} 中没有找到任何测试日志")
        return
    
    latest_log = os.path.join(log_dir, log_files[0])
    
    print("=" * 70)
    print(f"最新日志: {log_files[0]}")
    print("=" * 70)
    print()
    
    with open(latest_log, 'r') as f:
        content = f.read()
        print(content)

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-v':
        view_latest_log()
    else:
        list_test_logs()
        print()
        print("查看最新日志详情：")
        print("  python3 view_test_logs.py -v")
