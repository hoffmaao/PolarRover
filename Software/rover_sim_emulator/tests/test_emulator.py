def test_import():
    import rover_sim_emulator

    assert rover_sim_emulator.__version__


def test_cross_package_dependencies_resolve():
    import rover_drive  # noqa: F401
    import rover_sim  # noqa: F401
    import rover_sim_emulator  # noqa: F401


def test_cli_help_exits_zero():
    from rover_sim_emulator.cli import main

    assert main(["--help"]) == 0
