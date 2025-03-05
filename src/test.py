import pytest

def add(add1, add2):
    if (add1 + add2) > 10:
        return 10

    return add1 + add2

class TestClass:
    @pytest.mark.parametrize("add1,add2,expected",[
        (2,4,6),
        (3,3,6),
        (5,6,11)
    ])
    def test_add(self, add1, add2, expected):
        r = add(add1, add2)

        assert r == expected    