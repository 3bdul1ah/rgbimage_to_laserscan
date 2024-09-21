## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['depth_anything_v2'],
    package_dir={'': 'src/Depth_Anything_V2'},
)

setup(**setup_args)
