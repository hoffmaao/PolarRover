def test_import():
    import rover_sim_startup

    assert rover_sim_startup.__version__


def test_depends_on_rover_sim():
    import rover_sim  # noqa: F401


def test_cli_help_exits_zero():
    from rover_sim_startup.cli import main

    assert main(["--help"]) == 0
