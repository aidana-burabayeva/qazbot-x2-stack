from setuptools import find_packages, setup

package_name = 'x2_twin_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Digital Twin team',
    maintainer_email='burabayeva.aidana@gmail.com',
    description='Plant side of the X2 digital twin.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hal_plant = x2_twin_sim.plant.hal_plant_node:main',
            'kinematic_plant = x2_twin_sim.plant.kinematic_plant_node:main',
            'virtual_mc = x2_twin_sim.mc.virtual_mc_node:main',
            'truth_localizer = x2_twin_sim.loc.truth_localizer_node:main',
        ],
    },
)
