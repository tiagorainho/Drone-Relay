import threading

class ExitableThread(threading.Thread):
    def __init__(self,  *args, **kwargs):
        super(ExitableThread, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()

    def exit(self):
        self._stop_event.set()

    def exited(self):
        return self._stop_event.is_set()