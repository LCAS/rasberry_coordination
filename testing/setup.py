from setuptools import setup

package_name = 'rasberry_coordination_testing'

setup(
    name=package_name,
    version='1.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[''],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run.py = rasberry_coordination_testing.run:main',
            'routing_robustness.py = rasberry_coordination_testing.routing_robustness:main',
            'routing_comparison.py = rasberry_coordination_testing.routing_comparison:main'
        ],
    },
)





### what type or testing are we talking about?

# validate elements are working (github actions)
# robustness evaluations (wfc for map generation)
# comparison evaluations (wandb for analysis) (perhaps rosbags also)

