def test_public_imports():
    from rover_hardware import (
        FrameCoder,
        HardwareBackend,
        SingleTrackCANBackend,
        TankCANBackend,
        TankMixConfig,
        VehicleFeedback,
        mix_differential,
    )

    assert HardwareBackend is not None
    assert FrameCoder is not None
    assert SingleTrackCANBackend is not None
    assert TankCANBackend is not None
    assert TankMixConfig is not None
    assert VehicleFeedback is not None
    assert callable(mix_differential)
