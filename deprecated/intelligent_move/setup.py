from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'intelligent_move'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'sensor_msgs',
        'cv_bridge',
        'opencv-python',
        'numpy',
        'torch',
        'torchvision',
        'ultralytics',
        'openai',
        'SpeechRecognition',
        'pyaudio',
        'pyttsx3',
        'transformers',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Intelligent robot movement with voice control and object detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intelligent_move_node = intelligent_move.intelligent_move_node:main',
        ],
    },
)