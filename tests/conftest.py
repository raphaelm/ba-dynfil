import rbdl
import pytest


@pytest.fixture
def model():
    filename = "./data/models/ik_test.lua"
    return rbdl.loadModel(filename)
