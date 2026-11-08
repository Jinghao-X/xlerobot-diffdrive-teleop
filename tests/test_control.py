"""Two pages connected at once: one driving, one stale tab sending stop heartbeats.

Run:  python tests/test_control.py
"""

from __future__ import annotations

import asyncio
import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import joystick_server as js                                   # noqa: E402
from control_arbiter import ControlArbiter                      # noqa: E402
from test_base import CFG, Clock, make                          # noqa: E402


class FakeSocket:
    """Minimal stand-in for a websockets connection: yields scripted messages."""

    def __init__(self, ip: str, messages: list[tuple[float, str]]) -> None:
        self.remote_address = (ip, 50000)
        self.messages = messages          # (delay_s, json text)

    def __aiter__(self):
        return self._gen()

    async def _gen(self):
        for delay, text in self.messages:
            await asyncio.sleep(delay)
            yield text


def test_arbiter_rules():
    clock = Clock()
    a = ControlArbiter(idle_s=0.5, clock=clock)
    phone, stale = object(), object()
    assert a.accept(stale, 0, 0) == (True, True)       # nobody in control: first message takes it
    assert a.accept(phone, 0.0, 0.8) == (True, True)    # pushing the stick takes control
    for _ in range(50):
        clock.t += 0.05
        assert a.accept(phone, 0.0, 0.8) == (True, False)
        assert a.accept(stale, 0, 0) == (False, False)   # stale stop heartbeat ignored
    assert a.accept(phone, 0, 0) == (True, False)       # the controller's own stop always works
    clock.t += 0.6                                     # controller quiet
    assert a.accept(stale, 0, 0) == (True, True)
    a.disconnect(stale)
    assert a.controller is None


def stale_tab_scenario(use_arbiter: bool) -> list[float]:
    """Drive forward from one page for 3 s while a stale page sends stop every 1 s.

    Returns the base's left-wheel target sampled every 50 ms.
    """
    base, ser, clock = make()
    js.base, js.cfg = base, copy.deepcopy(CFG)
    js.clients.clear()
    js.arbiter = ControlArbiter() if use_arbiter else _NoArbiter()
    forward = '{"x": 0, "y": 0.8}'
    stop = '{"x": 0, "y": 0}'
    active = FakeSocket("192.168.1.50", [(0.02, forward)] + [(0.1, forward)] * 30)
    stale = FakeSocket("192.168.1.50", [(0.0, stop)] + [(1.0, stop)] * 3)
    samples: list[float] = []

    async def sample():
        for _ in range(60):
            await asyncio.sleep(0.05)
            samples.append(base.target[1])

    async def main():
        await asyncio.gather(js.handler(stale), js.handler(active), sample())
    asyncio.run(main())
    return samples[4:]                                 # skip the first moments before driving starts


class _NoArbiter(ControlArbiter):
    """The old behaviour: every message from every page is applied."""

    def accept(self, client, x, y):
        return True, False


def test_stale_tab_used_to_cause_stutter():
    samples = stale_tab_scenario(use_arbiter=False)
    assert any(t == 0 for t in samples), "expected the old behaviour to stop the wheels"


def test_stale_tab_ignored_now():
    samples = stale_tab_scenario(use_arbiter=True)
    assert samples and all(t > 0 for t in samples), samples


if __name__ == "__main__":
    import contextlib
    import io
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        with contextlib.redirect_stdout(io.StringIO()):
            fn()
        print("pass", name)
    print(f"OK: {len(tests)} tests")
