from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=[
        "asmc_perception",
        "asmc_perception.bev",
        "asmc_perception.camera",
        "asmc_perception.camera.backends",
        "asmc_perception.nodes",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
