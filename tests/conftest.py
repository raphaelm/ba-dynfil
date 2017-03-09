import pytest

from dynfil.models import SimpleModel, HeiCubModel
from tests.kinematics import minimal_trajectory


@pytest.fixture
def model():
    return SimpleModel()


@pytest.fixture
def heicub():
    return HeiCubModel()


@pytest.fixture(params=["analytical", "numerical"])
def ik_method(request):
    return request.param


@pytest.fixture(params=['heicub', 'simple'])
def any_model_with_trajectory(request):
    if request.param == "heicub":
        m = HeiCubModel()
        return m, minimal_trajectory(m, 'test_traj_heicub.txt')
    elif request.param == "simple":
        m = SimpleModel()
        return m, minimal_trajectory(m, 'test_traj_simple.txt')
