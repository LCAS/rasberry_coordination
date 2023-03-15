from setuptools import setup

package_name = 'rasberry_coordination_core'

setup(
    name=package_name,
    version='1.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rospy-message-converter'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_agent.py = rasberry_coordination_core.add_agent:main',
            'coordinator.py = rasberry_coordination_core.coordinator:main',
            'agent_markers.py = rasberry_coordination_core.agent_markers:main',
            'speaker.py = rasberry_coordination_core.speaker:main'
            # https://stackoverflow.com/a/782984/8929350
        ],
    },

)

