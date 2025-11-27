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
    install_requires=['setuptools'],
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
            'image_tools = uav_vision.image_tools:main'
        ],
    },
)
