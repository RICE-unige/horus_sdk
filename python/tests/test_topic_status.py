import io
from contextlib import redirect_stdout

from horus.utils.topic_status import TopicStatusBoard


def test_topic_status_non_tty_sub_unsub_tf(monkeypatch):
    monkeypatch.setattr("sys.stdout.isatty", lambda: False)
    board = TopicStatusBoard()

    buf = io.StringIO()
    with redirect_stdout(buf):
        board.on_subscribe("/tf")
        board.on_unsubscribe("/tf")

    lines = [line for line in buf.getvalue().splitlines() if "Topic:" in line]
    assert len(lines) == 2, f"unexpected lines: {lines}"
    assert "Topic: /tf" in lines[0] and (
        "Subscribed" in lines[0] or "SUBSCRIBED" in lines[0]
    )
    assert "Topic: /tf" in lines[1] and (
        "Unsubscribed" in lines[1] or "UNSUBSCRIBED" in lines[1]
    )
