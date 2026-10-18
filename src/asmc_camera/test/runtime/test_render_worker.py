import threading
import time
import unittest
from asmc_camera.runtime.render_worker import RenderWorker


class RenderWorkerTest(unittest.TestCase):
    def test_arrival_wakes_worker_without_periodic_polling(self):
        done=threading.Event(); worker=RenderWorker(done.set)
        self.addCleanup(worker.close)
        worker.offer(); self.assertTrue(done.wait(.5))

    def test_rate_wait_reads_latest_input_and_retains_one_pending_notification(self):
        entered=threading.Event(); complete=threading.Event(); values=[]; latest=[0]
        def render():
            values.append(latest[0]); entered.set()
            if len(values)==2:complete.set()
            return True
        worker=RenderWorker(render, max_hz=10)
        self.addCleanup(worker.close)
        worker.offer(); self.assertTrue(entered.wait(.5))
        for i in range(1,100):latest[0]=i; worker.offer()
        self.assertTrue(complete.wait(.5));self.assertEqual(values[:2],[0,99])
        self.assertLessEqual(len(values),2)

    def test_close_interrupts_long_rate_wait(self):
        rendered=threading.Event();worker=RenderWorker(lambda:rendered.set(), max_hz=.1)
        worker.offer();self.assertTrue(rendered.wait(.5));worker.offer()
        started=time.monotonic();worker.close()
        self.assertLess(time.monotonic()-started,.5)
        self.assertFalse(worker.thread.is_alive());self.assertFalse(worker.offer())

    def test_error_is_reported_and_next_arrival_can_render(self):
        reported=threading.Event();recovered=threading.Event();count=[0]
        def render():
            count[0]+=1
            if count[0]==1:raise ValueError('fixture')
            recovered.set()
        worker=RenderWorker(render,on_error=lambda error:reported.set())
        self.addCleanup(worker.close)
        worker.offer();self.assertTrue(reported.wait(.5))
        worker.offer();self.assertTrue(recovered.wait(.5))


if __name__=='__main__':unittest.main()
