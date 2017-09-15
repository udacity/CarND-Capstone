import datetime
import monotonic

def format_seconds(secs, coarse):
    if (coarse):
        secs = int(secs)
    delta = datetime.timedelta(seconds=secs)
    return str(delta)

def format_now():
    return datetime.datetime.now().strftime('%Y-%m-%d_%Hh%Mm%S')

class Stopwatch:
    def __init__(self, autostart = True):
        self._duration = 0
        self._started = False
        self._start = 0
        if (autostart):
            self.start()

    def start(self):
        if not self._started:
            self._started = True
            self._start = monotonic.monotonic()
        return self

    def stop(self):
        if not self._started:
            raise RuntimeError('Cannot stop stopwatch that has not been started.')
        end = monotonic.monotonic()
        self._duration += (end - self._start)
        self._started = False
        self._start = 0

    def reset(self):
        self._duration = 0
        self._started = False
        self._start = 0

    def format_duration(self, coarse = True):
        return format_seconds(self._duration, coarse)
