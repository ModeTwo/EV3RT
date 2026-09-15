"""Stop once while the backend receiver is alive; bound native API waits."""
import threading


class Shutdown:
    def __init__(self, stop, timeout=2.0):
        self.stop_callback = stop
        self.timeout = timeout
        self.requested = False
        self.errors = []

    def stop(self, reason):
        # Called on the main thread, including from a signal handler. Latch
        # before any I/O so a repeated signal cannot send commands twice.
        if self.requested:
            return self.errors
        self.requested = True
        print(' -- SHUTDOWN motor stop begin: ' + reason, flush=True)

        def run():
            try:
                self.errors.extend(self.stop_callback())
            except Exception as error:
                self.errors.append(str(error))

        worker = threading.Thread(target=run, name='motor-stop', daemon=True)
        worker.start()
        worker.join(self.timeout)
        if worker.is_alive():
            self.errors.append('motor stop timed out; physical stop not confirmed')
        print(' -- SHUTDOWN motor stop result: ' +
              ('; '.join(self.errors) if self.errors else 'completed'), flush=True)
        return self.errors
