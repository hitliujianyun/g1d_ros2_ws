import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from g1d_core.chassis_http import ChassisHttp, EventTracker


@pytest.fixture
def server():
    state = {"events": [], "wrap": False, "deleted": 0}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _reply(self, obj):
            body = json.dumps(obj).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802
            ev = list(state["events"])
            self._reply({"events": ev} if state["wrap"] else ev)

        def do_DELETE(self):  # noqa: N802
            state["deleted"] += 1
            self._reply({})

    srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield state, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def test_event_tracker_reports_only_new_events():
    tr = EventTracker()
    old = [{"type": "BUMPER_TRIGGERED", "timestamp": 1}]
    assert tr.update(old) == []                       # history at start-up never latches
    assert tr.update(old) == []
    new = old + [{"type": "CLIFF_DETECTED", "timestamp": 2}]
    assert tr.update(new) == ["CLIFF_DETECTED"]
    assert tr.update(new) == []


def test_http_events_list_and_wrapped(server):
    state, url = server
    http = ChassisHttp(url, timeout=2.0)
    state["events"] = [{"type": "A", "timestamp": 1}, "junk"]
    assert http.events() == [{"type": "A", "timestamp": 1}]
    state["wrap"] = True
    assert http.events() == [{"type": "A", "timestamp": 1}]
    assert http.abort_current_action() is True
    assert state["deleted"] == 1


def test_http_unreachable_abort_is_best_effort():
    assert ChassisHttp("http://127.0.0.1:9", timeout=0.2).abort_current_action() is False
