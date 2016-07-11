from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d= generate_distutils_setup(
    packages = ["robot_chassis"],
    scripts = ["scripts/robot_chassis"],
    package_dir = {'':'src'}
    )
setup(**d)
