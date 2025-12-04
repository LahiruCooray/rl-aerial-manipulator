from setuptools import find_packages, setup

package_name = 'manipulator_py_examples'

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
    maintainer='rajitha-niroshan',
    maintainer_email='nrajitha78@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_service_server = manipulator_py_examples.simple_service_server:main',
            'simple_service_client = manipulator_py_examples.simple_service_client:main',
            'simple_action_server = manipulator_py_examples.simple_action_server:main',
            'simple_action_client = manipulator_py_examples.simple_action_client:main',
        ],
    },
)
