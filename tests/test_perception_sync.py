from g1_backend.perception_sync import CaptureQueue


def test_delayed_tf_uses_newest_ready_capture_without_dropping_future_frames():
    queue = CaptureQueue()
    for stamp in [1.0, 1.1, 1.2]:
        queue.push(str(stamp), stamp, 10)
    chosen = queue.select(1.25, 10.1, lambda pair, stamp: stamp <= 1.1)
    assert chosen.stamp == 1.1 and chosen.pair == '1.1'
    assert [frame.stamp for frame in queue.queue] == [1.2]
    assert queue.select(1.3, 10.15, lambda pair, stamp: True).stamp == 1.2


def test_missing_tf_expires_and_never_replays_old_or_clock_reset_frames():
    queue = CaptureQueue(tf_wait=0.2)
    queue.push('missing', 1, 0)
    assert queue.select(1.1, 0.1, lambda *_: False) is None
    assert queue.select(1.2, 0.3, lambda *_: True) is None
    assert queue.dropped['tf_timeout'] == 1
    queue.push('ready', 2, 0.4)
    assert queue.select(2, 0.4, lambda *_: True).pair == 'ready'
    queue.push('late', 1.9, 0.5)
    assert queue.select(2.1, 0.5, lambda *_: True) is None
    queue.push('reset', 0.1, 0.6)
    assert queue.select(0.2, 0.6, lambda *_: True) is None
    queue.push('new', 0.3, 0.7)
    assert queue.select(0.3, 0.7, lambda *_: True).pair == 'new'


def test_queue_is_bounded_and_old_capture_not_accepted_even_with_tf():
    queue = CaptureQueue(capacity=2)
    for stamp in [1, 2, 3]:
        queue.push(stamp, stamp, 0)
    assert len(queue.queue) == 2 and queue.dropped['capacity'] == 1
    assert queue.select(3, 0, lambda *_: True).stamp == 3
    assert queue.dropped['stale'] == 1


def test_ready_capture_does_not_expire_while_waiting_for_inference_cadence():
    queue = CaptureQueue(tf_wait=0.2, max_age=0.5)
    queue.push('ready', 1, 0)
    assert queue.select(1.05, 0.05, lambda *_: True, consume=False).pair == 'ready'
    assert queue.last_processed < 0 and len(queue.queue) == 1
    assert queue.select(1.3, 0.3, lambda *_: False).pair == 'ready'
    assert queue.dropped['tf_timeout'] == 0
