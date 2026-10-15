"""Debug input subscriptions follow connected output demand using wall time."""
import threading
import message_filters
import rospy


class DemandSubscriber(message_filters.SimpleFilter):
    def __init__(self, topic, message_type, demand, callback=None, **kwargs):
        super().__init__()
        self.topic, self.message_type, self.demand = topic, message_type, demand
        self.kwargs, self.sub, self.closed = kwargs, None, False
        self.lock = threading.Lock()
        if callback is not None:
            self.registerCallback(callback)

    def refresh(self):
        with self.lock:
            wanted = not self.closed and self.demand()
            if wanted and self.sub is None:
                self.sub = rospy.Subscriber(self.topic,self.message_type,self.signalMessage,**self.kwargs)
            elif not wanted and self.sub is not None:
                self.sub.unregister()
                self.sub = None

    def unregister(self):
        with self.lock:
            self.closed = True
            if self.sub is not None:
                self.sub.unregister()
                self.sub = None


class DemandInputs:
    def __init__(self, inputs):
        self.inputs, self.stop = inputs, threading.Event()
        self.worker = threading.Thread(target=self._worker,name='debug-demand',daemon=True)
        rospy.on_shutdown(self.close)
        self.worker.start()

    def _worker(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            for subscriber in self.inputs:
                subscriber.refresh()
            self.stop.wait(.2)

    def close(self):
        self.stop.set()
        if threading.current_thread() is not self.worker:
            self.worker.join(timeout=1.)
        for subscriber in self.inputs:
            subscriber.unregister()


class DemandSynchronizer:
    """An exact-stamp subscription group with a fresh queue on every epoch.

    A disconnected ROS callback can still be in flight. Its captured epoch is
    checked before feeding the synchronizer and again before delivering a pair.
    The lock serializes disconnect with delivery; no old pair is delivered after
    refresh/unregister returns. ``context`` must be a cheap immutable snapshot.
    """
    def __init__(self, topics, message_type, demand, callback, context=lambda: None,
                 on_reset=None, queue_size=4, subscriber_queue_size=2, subscriber_buffer_bytes=None):
        self.topics, self.message_type = tuple(topics), message_type
        self.demand, self.callback, self.context = demand, callback, context
        self.on_reset = on_reset
        self.queue_size, self.subscriber_queue_size = queue_size, subscriber_queue_size
        self.subscriber_buffer_bytes = subscriber_buffer_bytes
        self.lock = threading.RLock()
        self.subscribers, self.synchronizer = [], None
        self.closed, self.active, self.epoch = False, False, 0
        self.current_context = None

    def _disconnect(self):
        self.epoch += 1
        self.active = False
        subscribers, self.subscribers = self.subscribers, []
        self.synchronizer = None
        for subscriber in subscribers:
            subscriber.unregister()
        if self.on_reset:
            self.on_reset()

    def refresh(self):
        with self.lock:
            wanted = not self.closed and bool(self.demand())
            context = self.context()
            if self.active and (not wanted or context != self.current_context):
                self._disconnect()
            if not wanted or self.active:
                return
            self.current_context = context
            epoch = self.epoch
            filters = [message_filters.SimpleFilter() for _ in self.topics]
            sync = message_filters.TimeSynchronizer(filters, self.queue_size)
            def deliver(*messages):
                # Called synchronously inside receive's epoch lock.
                if self.active and epoch == self.epoch and self.context() == context and self.demand():
                    self.callback(*messages)
            sync.registerCallback(deliver)
            self.synchronizer, self.active = sync, True
            for topic, source in zip(self.topics, filters):
                def receive(message, source=source):
                    with self.lock:
                        if self.active and epoch == self.epoch and self.context() == context and self.demand():
                            source.signalMessage(message)
                kwargs = dict(queue_size=self.subscriber_queue_size)
                if self.subscriber_buffer_bytes is not None:
                    kwargs['buff_size'] = self.subscriber_buffer_bytes
                self.subscribers.append(rospy.Subscriber(topic, self.message_type, receive, **kwargs))

    def unregister(self):
        with self.lock:
            self.closed = True
            self._disconnect()
