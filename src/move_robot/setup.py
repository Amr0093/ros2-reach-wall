from setuptools import setup

package_name = 'move_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='go',
    maintainer_email='go@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # format: "<command> = <package>.<python_file>:<function>"
            'action_server = move_robot.action_server:main',
            'action_client = move_robot.action_client:main',
        ],
    },
)
