from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['choosing_wait_position'],
    package_dir={'': 'src'}
)

setup(**d)
