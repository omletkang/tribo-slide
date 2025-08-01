from setuptools import find_packages, setup

package_name = 'tribo_plot'

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
    maintainer='kang',
    maintainer_email='kang@todo.todo',
    description='Real-time plotting for sensor data using PyQtGraph',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'sensorT_fake = tribo_plot.sensorT_fake:main',
            'plot_sensor = tribo_plot.plot_sensor:main',
            'models = tribo_plot.models:main',
            'app1 = tribo_plot.app1:main',
        ],
    },
)
