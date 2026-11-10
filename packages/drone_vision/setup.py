from setuptools import find_packages, setup

package_name = "drone_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Drone Vision Project",
    maintainer_email="user@example.invalid",
    description="Read-only visual perception nodes for the Duckiedrone simulator.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_probe = drone_vision.camera_probe:main",
            "tag_detector = drone_vision.tag_detector:main",
            "yolo_detector = drone_vision.yolo_detector:main",
            "safe_hover = drone_vision.safe_hover:main",
        ],
    },
)
