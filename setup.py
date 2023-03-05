from setuptools import setup

package_name = 'rasberry_coordination'

setup(
    name=package_name,
    version='1.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_agent = rasberry_coordination.ros2_nodes.add_agent:main',
            'ui_speaker = rasberry_coordination.ros2_nodes.ui_speaker:main',
            'abstract_task_executor_node.py = rasberry_coordination.ros2_nodes.abstract_task_executor_node:main'
            # https://stackoverflow.com/a/782984/8929350
        ],
    },

)

