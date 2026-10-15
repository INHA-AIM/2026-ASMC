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
