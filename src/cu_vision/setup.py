from setuptools import setup

package_name = "cu_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/target_detector.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="OpenCV based target detector for fixed-wing drop task.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_detector = cu_vision.target_detector:main",
        ],
    },
)

