"""Portable filesystem locations shared by the STEMM navigation stack."""

import os
from pathlib import Path


def data_home() -> Path:
    configured = os.getenv('XDG_DATA_HOME', '').strip()
    return Path(configured).expanduser() if configured else Path.home() / '.local' / 'share'


def map_directory() -> Path:
    configured = os.getenv('STEMM_MAP_DIR', '').strip()
    return Path(configured).expanduser() if configured else data_home() / 'stemm' / 'maps'


def map_basename(name: str = 'stemm_map') -> Path:
    return map_directory() / name
