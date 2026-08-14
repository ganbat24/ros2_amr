from setuptools import find_packages, setup

package_name = 'amr_metrics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Ganbat Selenge',
    maintainer_email='ganbat@example.com',
    description='AMR validation and metrics: trajectory recording, readiness gate, graphs',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'record_trajectory = amr_metrics.record_trajectory:main',
            'ready_gate = amr_metrics.ready_gate:main',
            'plot_metrics = amr_metrics.plot_metrics:main',
            'run_validation = amr_metrics.run_validation:main',
            'orchestrate = amr_metrics.orchestrate:main',
            'scan_health = amr_metrics.scan_health:main',
            'motion_health = amr_metrics.motion_health:main',
            'path_health = amr_metrics.path_health:main',
        ],
    },
)
