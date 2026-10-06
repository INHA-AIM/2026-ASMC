from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=[
        "asmc_camera",
        "asmc_camera.bev",
        "asmc_camera.camera",
        "asmc_camera.camera.backends",
        "asmc_camera.detection",
        "asmc_camera.nodes",
        "asmc_camera.offline",
        "asmc_camera.offline.bev",
        "asmc_camera.training",
        "asmc_camera.training.twinlite",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
