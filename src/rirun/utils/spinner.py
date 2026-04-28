import sys
import time
import threading
import logging
from itertools import cycle


class SpinnerStdoutWrapper:
    """Wraps stdout to pause spinner on any write operation."""

    def __init__(self, original_stdout, spinner):
        self.original_stdout = original_stdout
        self.spinner = spinner
        self._buffer = ""

    def write(self, text):
        # Buffer the text until we see a newline or it's substantial
        self._buffer += text

        # Check if we have a complete line or substantial content
        if "\n" in self._buffer or len(self._buffer) > 100:
            # Only pause/clear if there's actual content (not just empty strings or whitespace)
            content_to_write = self._buffer
            self._buffer = ""

            if content_to_write.strip():
                was_running = self.spinner.running and not self.spinner.paused
                if was_running:
                    self.spinner.pause()

                self.original_stdout.write(content_to_write)
                self.original_stdout.flush()

                if was_running:
                    # Small delay to ensure output is visible
                    time.sleep(0.02)
                    self.spinner.resume()
            else:
                self.original_stdout.write(content_to_write)
                self.original_stdout.flush()

    def flush(self):
        # Flush any remaining buffer
        if self._buffer:
            content = self._buffer
            self._buffer = ""
            if content.strip():
                was_running = self.spinner.running and not self.spinner.paused
                if was_running:
                    self.spinner.pause()

                self.original_stdout.write(content)
                self.original_stdout.flush()

                if was_running:
                    time.sleep(0.02)
                    self.spinner.resume()
            else:
                self.original_stdout.write(content)
                self.original_stdout.flush()
        else:
            self.original_stdout.flush()

    def __getattr__(self, name):
        return getattr(self.original_stdout, name)


class SpinnerLogHandler(logging.Handler):
    """
    Custom logging handler that pauses the spinner during log output.
    """

    def __init__(self, spinner, stream=None):
        super().__init__()
        self.spinner = spinner
        self.stream = stream or sys.stderr

    def emit(self, record):
        """Pause spinner, emit log, resume spinner."""
        was_running = self.spinner.running and not self.spinner.paused

        if was_running:
            self.spinner.pause()

        try:
            msg = self.format(record)
            self.stream.write(msg + "\n")
            self.stream.flush()
        except Exception:
            self.handleError(record)
        finally:
            if was_running:
                # Small delay to ensure log is visible
                time.sleep(0.05)
                self.spinner.resume()


class Spinner:
    """
    A loading spinner that works with Python logging and print statements.
    Automatically pauses when output is written to stdout or via logging.
    """

    def __init__(self, message="Loading", spinner_chars="⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏", autostart=False):
        self.message = message
        self.spinner_chars = cycle(spinner_chars)
        self.running = False
        self.paused = False
        self.thread = None
        self._lock = threading.Lock()
        self._original_stdout = None
        self._wrapped_stdout = None
        if autostart:
            self.start()

    def _spin(self):
        """Internal method that runs the spinning animation."""
        while self.running:
            if not self.paused:
                with self._lock:
                    char = next(self.spinner_chars)
                    # Bold text using ANSI escape codes: \033[1m for bold, \033[0m to reset
                    self._original_stdout.write(
                        f"\r  {char} \033[1m{self.message}\033[0m"
                    )
                    self._original_stdout.flush()
            time.sleep(0.1)

    def start(self):
        """Start the loading spinner animation and wrap stdout."""
        if self.running:
            return

        # Store original stdout and wrap it
        self._original_stdout = sys.stdout
        self._wrapped_stdout = SpinnerStdoutWrapper(self._original_stdout, self)
        sys.stdout = self._wrapped_stdout

        self.running = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop the loading spinner animation, clear the line, and restore stdout."""
        if not self.running:
            return

        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()

        # Clear the spinner line (account for 2 spaces + spinner + space + message)
        with self._lock:
            self._original_stdout.write("\r" + " " * (len(self.message) + 4) + "\r")
            self._original_stdout.flush()

        # Restore original stdout
        if self._original_stdout:
            sys.stdout = self._original_stdout
            self._original_stdout = None
            self._wrapped_stdout = None

    def pause(self):
        """Temporarily pause the spinner and clear its line."""
        if not self.running or self.paused:
            return

        self.paused = True
        with self._lock:
            # Account for 2 spaces prefix + spinner char + space + message (bold codes don't take visual space)
            self._original_stdout.write("\r" + " " * (len(self.message) + 4) + "\r")
            self._original_stdout.flush()

    def resume(self):
        """Resume the spinner after pausing."""
        if not self.running:
            return
        self.paused = False

    def update_message(self, new_message):
        """Update the spinner message while it's running."""
        with self._lock:
            # Clear the old message first (account for 2 spaces + spinner + space + message)
            if self.running and not self.paused:
                self._original_stdout.write("\r" + " " * (len(self.message) + 4) + "\r")
            self.message = new_message
            # Immediately show the new message if running
            if self.running and not self.paused:
                char = next(self.spinner_chars)
                self._original_stdout.write(f"\r  {char} \033[1m{self.message}\033[0m")
                self._original_stdout.flush()

    def setup_logging(self, level=logging.INFO, format="%(levelname)s: %(message)s"):
        """
        Convenience method to set up logging with the spinner-aware handler.
        Call this after creating the spinner but before starting it.
        """
        logger = logging.getLogger()
        logger.setLevel(level)

        # Remove existing handlers
        logger.handlers.clear()

        # Add spinner-aware handler
        handler = SpinnerLogHandler(self)
        handler.setFormatter(logging.Formatter(format))
        logger.addHandler(handler)

    def __enter__(self):
        """Context manager entry - start the spinner."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - stop the spinner."""
        self.stop()


# Example usage
if __name__ == "__main__":
    # Create spinner
    spinner = Spinner("Processing data")

    # Set up logging BEFORE starting the spinner
    spinner.setup_logging(level=logging.INFO, format="%(levelname)s: %(message)s")

    # Start the spinner
    spinner.start()

    # Mix spinner with print statements and logging
    time.sleep(1)
    print("This is a regular print statement")
    time.sleep(1)
    logging.info("This is a log message - works now!")
    time.sleep(1)

    spinner.update_message("Loading files")
    time.sleep(1)
    print("Print during different spinner message")
    time.sleep(1)
    logging.warning("Warning message works too!")
    time.sleep(1)

    print("Multiple")
    logging.info("Mixed")
    print("outputs")
    logging.error("All work together!")
    time.sleep(1)

    spinner.stop()
    print("✓ All done!")
