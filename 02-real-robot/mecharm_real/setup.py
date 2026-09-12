# 【讲解】安装规则：把 mecharm_real/ 目录装成 Python 模块；launch/ 与 config/ 装进 share；
# entry_points 定义两个命令：real_driver -> real_driver.py 的 main()，teach -> teach.py 的 main()。
import os
from glob import glob
from setuptools import setup

package_name = 'mecharm_real'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotic-group',
    maintainer_email='1990292743abc@gmail.com',
    description='mechArm 270 真机驱动与示教工具（与仿真共用任务节点）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'real_driver = mecharm_real.real_driver:main',
            'teach = mecharm_real.teach:main',
        ],
    },
)
