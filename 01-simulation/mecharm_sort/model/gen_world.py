#!/usr/bin/env python3
# 【讲解】仿真世界生成器：读 config/grid.yaml，把桌子、网格黑线、目标方块、俯视相机写成 SDF（桌上没有盒子，分类区是网格外空桌面）。
# 这样网格/料盒/相机的坐标只维护一份（grid.yaml），世界文件和任务节点永远一致。
# 用法：python3 gen_world.py ../config/grid.yaml normal sort_world.sdf
#       python3 gen_world.py ../config/grid_abnormal.yaml abnormal sort_world_abnormal.sdf
# 场景：normal   = 6 格各一个方块（绿红蓝各 2），全部可分类
#       abnormal = 1 绿(注入一次夹取失败) 2 黄(未识别) 3 蓝 4 空格 5 红 6 红(格子挪到不可达处)
import sys

import yaml

COLORS = {
    'red': (0.85, 0.10, 0.10), 'green': (0.10, 0.70, 0.15), 'blue': (0.10, 0.20, 0.85),
    'yellow': (0.90, 0.80, 0.10),
}
SCENARIOS = {
    'normal': {1: 'green', 2: 'red', 3: 'blue', 4: 'blue', 5: 'green', 6: 'red'},
    'abnormal': {1: 'green', 2: 'yellow', 3: 'blue', 5: 'red', 6: 'red'},
}


def mat(rgb):
    r, g, b = rgb
    return f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>'


def box_visual(name, size, pose, rgb):
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
            f'{mat(rgb)}</visual>')


def box_full(name, size, pose, rgb, mu=1.0):
    return (f'<collision name="{name}_c"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
            f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>'
            + box_visual(name, size, pose, rgb))


def main(grid_path, scenario, out_path):
    cfg = yaml.safe_load(open(grid_path, encoding='utf-8'))
    th, osz, cs = cfg['table_height'], cfg['object_size'], cfg['cell_size']
    cam = cfg['camera']
    parts = []
    parts.append(f'''<?xml version="1.0"?>
<!--
【讲解】桌面物体自动分类整理 仿真世界（{scenario} 场景）。由 model/gen_world.py 从 config/{grid_path.split('/')[-1]} 自动生成，不要手改，改 yaml 再生成。
- 系统插件：Physics 物理、SceneBroadcaster 场景广播、UserCommands 用户命令、Sensors 传感器（俯视相机渲染，ogre2）。
- table：灰色桌面（高 {th} m）；识别节点靠"高饱和度"分前景，桌面必须是低饱和的灰色。
- grid_marks：3×2 连成一片的网格黑线（只有视觉，无碰撞），格子中心坐标与 grid.yaml 一致。
- aruco_0..3：网格四角的 ArUco 定位码贴片，任务节点靠它们每次扫描时自动算像素→桌面映射（相机可任意摆放）。
- cube_N：25 mm 方块；真值统一走 /world/sort_world/dynamic_pose/info 桥接（不用逐个加 OdometryPublisher）。
- top_camera：俯视相机，位姿与 grid.yaml 的 camera 一致，发布 camera/image_raw 与 camera/camera_info。
-->
<sdf version="1.8">
  <world name="sort_world">
    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.72 0.72 0.72 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.1 0.1 -0.98</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>{mat((0.5, 0.5, 0.5))}</visual>
      </link>
    </model>

    <!-- 桌子：灰色实心块，桌面高 {th} m，机器人生成在桌面 (0,0,{th}) -->
    <model name="table">
      <static>true</static>
      <pose>0.1 0 {th / 2} 0 0 0</pose>
      <link name="table_link">
        {box_full('table', f'0.9 0.7 {th}', '0 0 0 0 0 0', (0.78, 0.78, 0.78))}
      </link>
    </model>
''')
    # 网格描边：每格画一个 cell_size 大小的黑框（格子尺寸 = 中心间距，相邻格共边，6 格连成一个 3×2 大网格），只有视觉
    lines = []
    csx, csy = (cs if isinstance(cs, (list, tuple)) else (cs, cs))
    z = th + 0.0003
    for cid, c in cfg['cells'].items():
        x, y = c['x'], c['y']
        for k, (dx, dy, sx, sy) in enumerate([(0, csy / 2, csx, 0.002), (0, -csy / 2, csx, 0.002),
                                               (csx / 2, 0, 0.002, csy), (-csx / 2, 0, 0.002, csy)]):
            lines.append(box_visual(f'cell{cid}_l{k}', f'{sx} {sy} 0.0006', f'{x + dx} {y + dy} {z} 0 0 0', (0.12, 0.12, 0.12)))
    parts.append('    <!-- 取物网格描边（3×2 连成一片，坐标同 grid.yaml） -->\n    <model name="grid_marks"><static>true</static><link name="l">\n      '
                 + '\n      '.join(lines) + '\n    </link></model>\n')
    # ArUco 网格定位码：贴在桌面上的薄片，贴图 markers/aruco_<id>.png（由 config/make_markers.py 生成）。
    # 顶面贴图方向：图片"上"= 桌面 +x（前方），图片"右"= 桌面 -y；程序只用码中心，方向不影响定位
    for mid, m in cfg.get('markers', {}).items():
        s_ = m['size'] * 1.25   # 含白边的总尺寸（贴图里码占 80%）
        parts.append(f'''    <model name="aruco_{mid}"><static>true</static><pose>{m['x']} {m['y']} {th + 0.0004} 0 0 0</pose><link name="l">
      <visual name="v"><geometry><box><size>{s_} {s_} 0.0008</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>model://mecharm_sort/markers/aruco_{mid}.png</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr>
        </material></visual>
    </link></model>
''')
    # 方块
    for cid, color in SCENARIOS[scenario].items():
        c = cfg['cells'][cid]
        rgb = COLORS[color]
        zc = th + osz / 2 + 0.001
        parts.append(f'''    <!-- 方块 cube_{cid}：{color}，初始在 {cid} 号格 -->
    <model name="cube_{cid}">
      <pose>{c['x']} {c['y']} {zc} 0 0 0</pose>
      <link name="body">
        <inertial><mass>0.05</mass><inertia><ixx>5.2e-6</ixx><iyy>5.2e-6</iyy><izz>5.2e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        {box_full('cube', f'{osz} {osz} {osz}', '0 0 0 0 0 0', rgb, 2.0)}
      </link>
    </model>
''')
    cx, cy, cz = cam['xyz']
    r, p, yw = cam['rpy']
    parts.append(f'''    <!-- 俯视相机：位姿同 grid.yaml camera（桌面左侧上方斜向下对准网格中心），10 Hz 1280x720（与真机 USB 相机相同），水平视场 1.0 rad -->
    <model name="top_camera">
      <static>true</static>
      <pose>{cx} {cy} {cz} {r} {p} {yw}</pose>
      <link name="link">
        <visual name="body"><pose>-0.02 0 0 0 0 0</pose><geometry><box><size>0.04 0.03 0.03</size></box></geometry>{mat((0.15, 0.15, 0.15))}</visual>
        <sensor name="camera" type="camera">
          <camera>
            <horizontal_fov>1.0</horizontal_fov>
            <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>5.0</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>false</visualize>
          <topic>camera/image_raw</topic>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
''')
    open(out_path, 'w', encoding='utf-8').write(''.join(parts))
    print(f'生成 {out_path}：{len(SCENARIOS[scenario])} 个方块，{len(cfg["cells"])} 格，分类区 {list(cfg["zones"])}（无盒子）')


if __name__ == '__main__':
    main(*sys.argv[1:4])
