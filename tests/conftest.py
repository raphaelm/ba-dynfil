import rbdl
import pytest

from dynfil.models import SimpleModel


@pytest.fixture
def model():
    return SimpleModel()
