from setuptools import setup

package_name = 'data_record'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/data_record.launch.py']),
        ('share/' + package_name + '/config', ['config/static_frames.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Static TF + dataset rosbag capture launcher for instrumented bike',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_from_yaml = data_record.static_tf_from_yaml:main',
        ],
    },
)