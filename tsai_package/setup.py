from setuptools import find_packages, setup

package_name = 'tsai_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='iclab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo11_publisher=tsai_package.yolo11_publisher:main',
            'yolo11_subscriber=tsai_package.yolo11_subscriber:main',
            'yolo11_sub_pub=tsai_package.yolo11_sub_pub:main',
        ],
    },
)
