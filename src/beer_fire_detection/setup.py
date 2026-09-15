from setuptools import setup

package_name = 'beer_fire_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=['beer_fire_detection'],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    author='FAIYAD',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'fire_detector = beer_fire_detection.fire_detector:main',
            'synthetic_thermal_demo = beer_fire_detection.synthetic_thermal_demo:main',
        ],
    },
)
