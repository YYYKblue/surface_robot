from dataclasses import fields
from pathlib import Path

import pytest
import yaml

from f1_pool_cleaning.config import Config


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def route_parameters():
    return yaml.safe_load((PACKAGE / "config/l_cleaning.yaml").read_text())["l_controller"]["ros__parameters"]


@pytest.fixture
def route_config(route_parameters):
    names = {field.name for field in fields(Config)}
    return Config(**{k: v for k, v in route_parameters.items() if k in names}).validate()
