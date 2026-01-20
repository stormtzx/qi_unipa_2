from setuptools import setup, find_packages
from glob import glob

package_name = 'qi_unipa_2'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),  # Trova automaticamente tutti i package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniele',
    maintainer_email='stormtzx@gmail.com',
    description='Pepper Unipa ROS2 Nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qi_unipa_2_sensor = qi_unipa_2.qi_unipa_2_sensor:main',
            'qi_unipa_2_movement = qi_unipa_2.qi_unipa_2_movement:main',
            'qi_unipa_2_speech = qi_unipa_2.qi_unipa_2_speech:main',
            'qi_unipa_2_server = qi_unipa_2.qi_unipa_2_server:main',
            'qi_unipa_2_vision = qi_unipa_2.qi_unipa_2_vision:main',
            'qi_unipa_2_audio = qi_unipa_2.qi_unipa_2_audio:main',
            'qi_unipa_2_reference = qi_unipa_2.qi_unipa_2_reference:main',
        ],
    },
)
