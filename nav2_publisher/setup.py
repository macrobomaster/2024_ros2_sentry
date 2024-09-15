from setuptools import find_packages, setup

package_name = 'nav2_publisher'
setup(
    name=package_name,
    version='0.0.0',
    packages=['nav2_publisher'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher = nav2_publisher.goal_publisher:main',
        ],
    },
)
