from horus.utils.topic_status import TopicStatusBoard


def test_board_state_transitions_non_tty(monkeypatch):
    # Force non-TTY behavior for deterministic test
    monkeypatch.setattr("sys.stdout.isatty", lambda: False)
    board = TopicStatusBoard(max_hz=100)

    # Initial subscribe
    board.on_subscribe("/robot1/camera/image_raw")
    # Immediate unsubscribe
    board.on_unsubscribe("/robot1/camera/image_raw")
    # Re-subscribe
    board.on_subscribe("/robot1/camera/image_raw")

    # Ensure no exceptions and internal state is SUBSCRIBED at the end
    assert "/robot1/camera/image_raw" in board._topics
    assert board._topics["/robot1/camera/image_raw"].state == "SUBSCRIBED"

    # Cleanup safe even if non-running
    board.stop()
