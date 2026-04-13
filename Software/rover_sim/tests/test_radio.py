import pytest

from rover_sim.sensors import RadioLink, RadioLinkConfig


def test_zero_latency_delivers_immediately():
    link = RadioLink(
        RadioLinkConfig(
            latency_ms=0.0, jitter_ms=0.0, loss_rate=0.0, min_interval_ms=0.0
        )
    )
    assert link.send(side="a", payload="hello", t=0.0) is True
    delivered = link.poll(side="b", t=0.0)
    assert delivered == ["hello"]
    assert link.poll(side="a", t=0.0) == []  # wrong side receives nothing


def test_latency_delays_delivery():
    link = RadioLink(
        RadioLinkConfig(
            latency_ms=100.0, jitter_ms=0.0, loss_rate=0.0, min_interval_ms=0.0
        )
    )
    link.send(side="a", payload="later", t=0.0)
    assert link.poll(side="b", t=0.05) == []
    assert link.poll(side="b", t=0.10) == ["later"]


def test_oversize_payload_rejected_at_tx():
    link = RadioLink(
        RadioLinkConfig(
            max_payload_bytes=32,
            latency_ms=0.0,
            jitter_ms=0.0,
            loss_rate=0.0,
            min_interval_ms=0.0,
        )
    )
    assert link.send(side="a", payload="x", t=0.0, size_bytes=33) is False
    assert link.poll(side="b", t=1.0) == []


def test_rate_limit_drops_bursts():
    link = RadioLink(
        RadioLinkConfig(
            latency_ms=0.0,
            jitter_ms=0.0,
            loss_rate=0.0,
            min_interval_ms=100.0,
        )
    )
    assert link.send(side="a", payload="p1", t=0.0) is True
    assert link.send(side="a", payload="p2", t=0.05) is False
    assert link.send(side="a", payload="p3", t=0.10) is True
    delivered = link.poll(side="b", t=0.2)
    assert delivered == ["p1", "p3"]


def test_loss_rate_statistics():
    link = RadioLink(
        RadioLinkConfig(
            latency_ms=0.0,
            jitter_ms=0.0,
            loss_rate=0.5,
            min_interval_ms=0.0,
            seed=123,
        )
    )
    sent = 0
    for i in range(1000):
        if link.send(side="a", payload=i, t=i * 0.001):
            sent += 1
    delivered = link.poll(side="b", t=1.0)
    assert len(delivered) == sent
    assert 400 < sent < 600  # ~50% ± noise over 1000 trials


def test_bidirectional_isolation():
    link = RadioLink(
        RadioLinkConfig(
            latency_ms=0.0, jitter_ms=0.0, loss_rate=0.0, min_interval_ms=0.0
        )
    )
    link.send(side="a", payload="a→b", t=0.0)
    link.send(side="b", payload="b→a", t=0.0)
    assert link.poll(side="b", t=0.0) == ["a→b"]
    assert link.poll(side="a", t=0.0) == ["b→a"]


def test_delivery_order_stable_under_jitter():
    link = RadioLink(
        RadioLinkConfig(
            latency_ms=50.0,
            jitter_ms=0.0,
            loss_rate=0.0,
            min_interval_ms=0.0,
        )
    )
    # With zero jitter, delivery order matches send order
    for i in range(5):
        link.send(side="a", payload=i, t=i * 0.01)
    out = link.poll(side="b", t=1.0)
    assert out == [0, 1, 2, 3, 4]


def test_invalid_side_raises():
    link = RadioLink()
    with pytest.raises(ValueError):
        link.send(side="c", payload=0, t=0.0)
    with pytest.raises(ValueError):
        link.poll(side="c", t=0.0)


def test_default_profile_is_wifi_lan():
    cfg = RadioLinkConfig()
    assert cfg.latency_ms < 10
    assert cfg.loss_rate < 0.01
    assert cfg.max_payload_bytes >= 1024


def test_named_profiles_are_distinct():
    wifi = RadioLinkConfig.wifi_lan()
    xbee = RadioLinkConfig.xbee_900()
    lora = RadioLinkConfig.lora_sf7()

    # Latency ordering: WiFi fastest, LoRa/XBee much slower
    assert wifi.latency_ms < xbee.latency_ms
    assert wifi.latency_ms < lora.latency_ms
    # Payload envelopes shrink as we move toward narrowband radios
    assert wifi.max_payload_bytes > xbee.max_payload_bytes > lora.max_payload_bytes


def test_profile_factory_accepts_overrides():
    cfg = RadioLinkConfig.wifi_lan(latency_ms=20.0, seed=42)
    assert cfg.latency_ms == 20.0
    assert cfg.seed == 42
    # Unset fields keep the profile defaults
    assert cfg.loss_rate < 0.01
