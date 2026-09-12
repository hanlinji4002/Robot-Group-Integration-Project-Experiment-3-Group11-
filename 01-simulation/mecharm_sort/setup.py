# 【讲解】安装规则。模块 mecharm_sort/（三个节点 + 运动学 + 真值读取）；数据文件 launch/ model/ config/ 分别装到
# share/mecharm_sort/{launch,urdf,worlds,config}；三个可执行入口：color_detector / pick_place_server / task_manager。
import os
from glob import glob
from setuptools import setup

package_name = 'mecharm_sort'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('model/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('model/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'markers'), glob('markers/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotic-group',
    maintainer_email='1990292743abc@gmail.com',
    description='实验三 桌面物体自动分类整理（仿真阶段）：识别 → 网格判断 → 抓取分类 → 异常处理',
    license='MIT',
    entry_points={
        'console_scripts': [
            'color_detector = mecharm_sort.color_detector:main',
            'pick_place_server = mecharm_sort.pick_place_server:main',
            'task_manager = mecharm_sort.task_manager:main',
            'yolo_detector = mecharm_sort.yolo_detector:main',
            'usb_camera = mecharm_sort.usb_camera:main',
            'teach_points = mecharm_sort.teach_points:main',
        ],
    },
)
