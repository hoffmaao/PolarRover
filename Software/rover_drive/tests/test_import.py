def test_import():
    import rover_drive

    assert rover_drive.__version__


def test_depends_on_rover_sim():
    import rover_sim  # noqa: F401
