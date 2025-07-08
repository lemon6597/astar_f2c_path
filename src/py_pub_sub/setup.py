from setuptools import find_packages, setup

package_name = 'py_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/py_pub_sub/launch',['launch/plan_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humble',
    maintainer_email='humble@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "talker = py_pub_sub.test_publisher:main",
            "Astar = py_pub_sub.Astar:main",
            "Start = py_pub_sub.Start:main",
        ],
    },
)
