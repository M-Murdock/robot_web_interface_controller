from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['robot_web_inteface_controller'],
    package_dir={'': 'src'}
)
setup(**d)