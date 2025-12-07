# =============================================================================
# DUAL LOGGER - Captures console output & saves as md eport    -> Author: Zahin
# =============================================================================
# Usage:
#   logger = DualLogger(prefix="robot_1")
#   logger.start()
#   print("This gets logged")
#   logger.stop()
#   logger.save("logs/report.md")
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
    
    def start(self):
        """Start capturing stdout."""
        self.original_stdout = sys.stdout
        sys.stdout = self
    
    def stop(self):
        """Stop capturing stdout and restore original."""
        if self.original_stdout:
            sys.stdout = self.original_stdout
        self.save()
    
    def save(self):
        """Save captured log to file."""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("logs", exist_ok=True) # Ensure logs folder exists for controller
        filename = os.path.join("logs", f"report_{timestamp}.md")
        
        with open(filename, "w", encoding='utf-8') as f:
            f.write("".join(self.log))
        
        print(f"Report saved: {filename}")
        self.log = [] # Clear log
        return filename
    
        