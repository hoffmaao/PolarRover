"""Text-to-speech backends.

Two implementations are provided. ``PyttsxTTS`` wraps the pyttsx3 library
(offline, uses espeak or an OS-provided voice) and is the default on the
field rover. ``NullTTS`` swallows the output silently — used in tests
and when the rover is deployed without a speaker.

The abstract base lets us swap in other backends (piper, coqui-tts) later
without touching the audio event dispatcher.
"""

from typing import Protocol, runtime_checkable


@runtime_checkable
class TTSBackend(Protocol):
    """Anything that can speak a string synchronously."""

    def say(self, text: str) -> None: ...
    def close(self) -> None: ...


class NullTTS:
    """No-op backend. The rover pretends to speak; tests assert on calls."""

    def __init__(self) -> None:
        self.utterances: list[str] = []

    def say(self, text: str) -> None:
        self.utterances.append(text)

    def close(self) -> None:
        pass


class PyttsxTTS:
    """Wraps pyttsx3 — offline TTS via espeak / SAPI / NSSpeechSynthesizer."""

    def __init__(
        self,
        rate_wpm: int = 180,
        volume: float = 0.9,
        voice_id: str | None = None,
    ) -> None:
        try:
            import pyttsx3
        except ImportError as e:
            raise RuntimeError(
                "PyttsxTTS requires the 'audio' extra. "
                "Install with: pip install rover_onboard[audio]"
            ) from e

        self._engine = pyttsx3.init()
        self._engine.setProperty("rate", rate_wpm)
        self._engine.setProperty("volume", max(0.0, min(1.0, volume)))
        if voice_id is not None:
            self._engine.setProperty("voice", voice_id)

    def say(self, text: str) -> None:
        if not text:
            return
        self._engine.say(text)
        self._engine.runAndWait()

    def close(self) -> None:
        try:
            self._engine.stop()
        except Exception:
            pass
