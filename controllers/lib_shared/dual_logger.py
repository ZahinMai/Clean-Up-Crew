# =============================================================================
# DUAL LOGGER - Captures console output & saves as md report   -> Author: Zahin
# =============================================================================
# Usage:
#   logger = Logger(prefix="robot_1")
#   logger.start("output.md")
#   logger.write("This gets logged")
#   logger.stop()
# =============================================================================
import sys, datetime, os

class Logger:
    def __init__(self, prefix="", enabled=True):
        """Initialise logger with optional prefix for all messages."""
        self.prefix = prefix
        self.enabled = enabled
        self.terminal = sys.stdout
        self.log = []
        self.original_stdout = None
        self.filename = None
        self.saved = False  # Track if we've already saved
    
    def write(self, message):
        """Write message to both console and memory."""
        # Add prefix if message is not just whitespace
        if self.prefix and message.strip():
            prefixed = f"[{self.prefix}] {message}"
        else:
            prefixed = message
        
        self.terminal.write(prefixed)
        if self.enabled:
            self.log.append(prefixed)
    
    def flush(self):
        """Flush the output stream (required for stdout compatibility)."""
        if hasattr(self.terminal, 'flush'):
            self.terminal.flush()
    
    def start(self, filename=None):
        """Start capturing stdout."""
        self.filename = filename
        self.saved = False
        self.original_stdout = sys.stdout
        sys.stdout = self
    
    def stop(self):
        """Stop capturing stdout and restore original."""
        if self.original_stdout:
            sys.stdout = self.original_stdout
            self.original_stdout = None  # Prevent calling stop() twice
            self.save()
    
    def save(self):
        """Save captured log to file."""
        # Don't save if already saved or log is empty
        if self.saved or not self.log:
            return None
        
        os.makedirs("logs", exist_ok=True)
        
        if not self.filename:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join("logs", f"report_{timestamp}.md")
        else:
            filename = os.path.join("logs", self.filename)
        
        with open(filename, "w", encoding='utf-8') as f:
            f.write("".join(self.log))
        
        # Restore stdout temporarily to print save message
        original = sys.stdout
        sys.stdout = self.terminal
        print(f"Report saved: {filename}")
        sys.stdout = original
        
        self.saved = True  # Mark as saved
        return filename