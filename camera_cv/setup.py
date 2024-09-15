from setuptools import setup

package_name = 'camera_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bill',
    maintainer_email='onbilllin@gmail',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = camera_cv.image_subscriber:main',
        ],
    },
)

