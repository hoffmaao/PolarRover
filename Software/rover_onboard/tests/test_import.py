def test_public_imports():
    from rover_onboard import (
        AudioEvent,
        BatteryStatus,
        DriveStatus,
        EventType,
        GnssStatus,
        RoverStatusSnapshot,
        StatusBroker,
        SurveyStatus,
        phrase_for,
    )

    assert AudioEvent is not None
    assert BatteryStatus is not None
    assert DriveStatus is not None
    assert EventType is not None
    assert GnssStatus is not None
    assert RoverStatusSnapshot is not None
    assert StatusBroker is not None
    assert SurveyStatus is not None
    assert callable(phrase_for)
