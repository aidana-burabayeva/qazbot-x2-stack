from setuptools import find_packages, setup

package_name = 'x2_twin_scenarios'

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
    description='Scenario runner and analysis tools for the X2 digital twin.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'live_chain = x2_twin_scenarios.live_chain:main',
            'i1_collision = x2_twin_scenarios.i1_collision:main',
            'leg_vs_lio = x2_twin_scenarios.analysis.leg_vs_lio:main',
            'replay_amcl = x2_twin_scenarios.analysis.replay_amcl:main',
            'summarize = x2_twin_scenarios.analysis.summarize:main',
        ],
    },
)
