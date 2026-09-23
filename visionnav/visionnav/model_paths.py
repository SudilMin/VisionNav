"""Where model weights live: <package>/models in the source tree (colcon --symlink-install),
otherwise the installed share/visionnav/models."""
import os

_SRC_MODELS = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "models")


def models_dir() -> str:
    if os.path.isdir(_SRC_MODELS):
        return _SRC_MODELS
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("visionnav"), "models")


def model_path(name: str) -> str:
    return os.path.join(models_dir(), name)
