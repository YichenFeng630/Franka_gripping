#!/usr/bin/env python3
"""
生成8-10个方块进行精度测试
"""

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel, DeleteModel
import time

# Cube SDF（来自spawn_cubes.py）
cube_sdf="""
<?xml version="1.0" ?>
<sdf version="1.4">
<model name='%NAME%'>
  <static>0</static>
  <link name='%NAME%'>
    <inertial>
        <mass>0.066</mass>
        <inertia>
          <ixx>0.0000221859</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.0000221859</iyy>
          <iyz>0.0</iyz>
          <izz>0.0000221859</izz>
        </inertia>
      </inertial>
    <collision name='collision'>
      <max_contacts>10</max_contacts>
      <surface>
        <contact>
          <ode>
            <max_vel>0</max_vel>
            <min_depth>0.003</min_depth>
          </ode>
        </contact>
        <friction>
          <ode>
            <mu>10</mu>
            <mu2>10</mu2>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <slip2>0</slip2>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>1e+06</threshold>
        </bounce>
      </surface>
      <geometry>
        <box>
          <size> 0.045 0.045 0.045 </size>
        </box>
      </geometry>
    </collision>
    <visual name='%NAME%'>
      <pose>0 0 0 0 0 0</pose>
      <geometry>
        <box>
          <size> 0.045 0.045 0.045 </size>
        </box>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>%COLOR%</name>
        </script>
      </material>
    </visual>
  </link>
  <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
    <frameName>world</frameName>
    <bodyName>%NAME%</bodyName>
    <topicName>%NAME%_odom</topicName>
    <updateRate>30.0</updateRate>
  </plugin>
</model>
"""

rospy.init_node('spawn_test_cubes', anonymous=True)

def delete_old_cubes():
    """删除旧的测试方块"""
    try:
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("gazebo/delete_model", timeout=2.0)
        
        # 删除旧的4个方块
        for i in range(4):
            color_names = ['RED', 'BLUE', 'GREEN', 'YELLOW']
            model_name = f'cube_{color_names[i]}_{i+1}'
            try:
                delete_model(model_name)
                rospy.loginfo(f"Deleted {model_name}")
            except:
                pass
    except:
        pass

def spawn_cubes_grid(num_cubes=8):
    """
    以网格模式生成方块
    """
    print(f"\n{'='*80}")
    print(f"📦 准备生成 {num_cubes} 个测试方块")
    print(f"{'='*80}\n")
    
    try:
        # 删除旧方块
        delete_old_cubes()
        time.sleep(1)
        
        # 等待spawn服务
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        rospy.wait_for_service("gazebo/spawn_sdf_model", timeout=5.0)
        
        # 定义颜色列表
        colors = ['RED', 'BLUE', 'GREEN', 'YELLOW', 'RED', 'BLUE', 'GREEN', 'YELLOW', 'RED', 'BLUE']
        
        # 计算网格布局（在桌面上放置方块）
        # 桌面范围：约0.3-0.7 in X, -0.3-0.3 in Y
        positions = [
            # 第一行 (X=0.35)
            [0.35, -0.25, 0.022],
            [0.35, 0.00, 0.022],
            [0.35, 0.25, 0.022],
            
            # 第二行 (X=0.50)
            [0.50, -0.25, 0.022],
            [0.50, 0.00, 0.022],
            [0.50, 0.25, 0.022],
            
            # 第三行 (X=0.65)
            [0.65, -0.25, 0.022],
            [0.65, 0.00, 0.022],
            [0.65, 0.25, 0.022],
            [0.65, -0.10, 0.022],  # 10个
        ]
        
        spawned_cubes = []
        
        for i in range(num_cubes):
            color_name = colors[i]
            model_name = f'cube_{color_name}_{i+1}'
            
            # 使用预定义位置
            position = positions[i]
            orientation = [0, 0, 0]  # 不旋转
            
            # 替换SDF中的颜色和名称
            model_sdf = cube_sdf.replace('%NAME%', model_name)
            model_sdf = model_sdf.replace('%COLOR%', color_name.lower())
            
            # 创建姿态
            cube_pose = Pose(Point(*position), Quaternion(*quaternion_from_euler(*orientation)))
            
            # Spawn方块
            spawn_model(model_name, model_sdf, "", cube_pose, "world")
            spawned_cubes.append((model_name, position))
            
            print(f"[{i+1}/{num_cubes}] Spawned {model_name} at [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.3f}]")
            time.sleep(0.1)
        
        print(f"\n✅ 成功生成 {len(spawned_cubes)} 个方块！\n")
        
        # 打印配置
        print(f"{'='*80}")
        print(f"方块配置信息:")
        print(f"{'='*80}\n")
        print("Gazebo方块信息:")
        for name, pos in spawned_cubes:
            print(f"  {name:20s} : [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
        print(f"\n{'='*80}")
        print(f"下一步：运行精度测试")
        print(f"{'='*80}\n")
        print("在另一个终端运行:")
        print("  cd /opt/ros_ws")
        print("  python3 src/franka_perception/scripts/test_accuracy.py")
        print()
        
        return spawned_cubes
        
    except rospy.ServiceException as e:
        print(f"❌ Gazebo Service error: {e}")
        return None
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

if __name__ == '__main__':
    # 生成8个方块
    spawn_cubes_grid(num_cubes=8)
