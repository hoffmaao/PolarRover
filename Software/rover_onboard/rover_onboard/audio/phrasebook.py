"""Event → spoken phrase mapping.

Keeping the phrases here rather than inline in the event producers means
we can tune, translate, or silence specific callouts without touching the
logic that decides when to say them. Phrase templates use {} placeholders
that match the payload keys in AudioEvent.
"""

from rover_onboard.audio.events import AudioEvent, EventType


_PHRASES: dict[EventType, str] = {
    EventType.MISSION_STARTED: "{rover_name} mission started.",
    EventType.WAYPOINT_REACHED: "Waypoint {index}.",
    EventType.MISSION_COMPLETE: "Survey complete.",
    EventType.GNSS_FIX_LOST: "GPS fix lost. Pausing.",
    EventType.GNSS_FIX_RECOVERED: "GPS fix recovered.",
    EventType.BATTERY_LOW: "Battery at {soc_percent} percent.",
    EventType.ESTOP_ENGAGED: "Emergency stop engaged.",
    EventType.ESTOP_CLEARED: "Emergency stop cleared.",
    EventType.OBSTACLE_AHEAD: "Obstacle ahead. Rerouting.",
    EventType.CARD_DETECTED: "Survey card detected.",
    EventType.NO_SURVEY: "No survey loaded. Standing by.",
    EventType.MODE_CHANGED: "Mode {to}.",
}


def phrase_for(event: AudioEvent) -> str:
    """Render the event into a short spoken phrase."""
    template = _PHRASES.get(event.type, "{type} event.")
    try:
        return template.format(type=event.type.value, **event.payload)
    except KeyError:
        # If the payload is missing a field, fall back to the bare template
        # so we never crash the audio path for a malformed event.
        return template
