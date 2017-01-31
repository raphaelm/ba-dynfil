import rbdl
import pytest


@pytest.fixture
def model():
    filename = "./data/models/iCubHeidelberg01_new_legs.urdf"
    return rbdl.loadModel(filename)
