from setuptools import find_packages, setup

package_name = 'beer_fire_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='FAIYAD',
    maintainer_email='faiyad@example.com',
    description='B.E.E.R. thermal fire detection node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fire_detector = beer_fire_detection.fire_detector:main',
            'synthetic_thermal_demo = beer_fire_detection.synthetic_thermal_demo:main',
        ],
    },
)
