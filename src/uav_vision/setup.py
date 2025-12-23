from setuptools import find_packages, setup

package_name = 'uav_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics<=8.2.70',
        'numpy<2.0.0',
        'opencv-python<=4.10.0.84',
    ],
    zip_safe=True,
    maintainer='mikelap',
    maintainer_email='mi.tamvak@gmail.com',
    description='UAV vision tools',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detection = uav_vision.object_detection:main'
        ],
    },
)
