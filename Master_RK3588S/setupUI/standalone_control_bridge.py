import signal
import threading
import time

from control_runtime import ControlRuntime


STATUS_INTERVAL_SECONDS = 2.0


def run():
    """Run reusable pose forwarding and temporary manual control for 1.07."""
    stop_event = threading.Event()

    def request_stop(_signum=None, _frame=None):
        stop_event.set()

    for signal_name in ("SIGINT", "SIGTERM"):
        signal_value = getattr(signal, signal_name, None)
        if signal_value is not None:
            signal.signal(signal_value, request_stop)

    runtime = ControlRuntime()
    runtime.start()
    print("Standalone I/O bridge started.")
    next_status_time = 0.0

    try:
        while not stop_event.is_set():
            now = time.time()
            if now >= next_status_time:
                status = runtime.snapshot()
                print(
                    "I/O "
                    f"source={status['source']} "
                    f"pose={status['pose']['packet_count']} "
                    f"gamepad={status['gamepad']['packet_count']} "
                    f"serial={'ON' if status['serial']['online'] else 'OFF'}"
                )
                next_status_time = now + STATUS_INTERVAL_SECONDS

            stop_event.wait(0.1)
    finally:
        runtime.stop()
        print("Standalone I/O bridge stopped.")


if __name__ == "__main__":
    run()
