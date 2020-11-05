
class EmergencyBreaker():
    def __init__(self):
        self._emergency_break = False
    def toggle(self):
        self._emergency_break = not self._emergency_break
    def enable(self):
        self._emergency_break = True
    def state(self):
        return self._emergency_break 