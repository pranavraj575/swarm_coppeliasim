import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame

class JoyStick_helper:
    def __init__(self, js_id=0):

        pygame.display.init()
        pygame.joystick.init()
        self.js = pygame.joystick.Joystick(js_id)
        
        self.dead = 0.1
        self.button_state = False
        
    def get_state(self):
        pygame.event.pump()
        llr = self.js.get_axis(0)
        lud = -self.js.get_axis(1)
        rlr = self.js.get_axis(3)
        rud = -self.js.get_axis(4)
        
        llr *= not (abs(llr) < self.dead)
        lud *= not (abs(lud) < self.dead)
        rlr *= not (abs(rlr) < self.dead)
        rud *= not (abs(rud) < self.dead)

        llr += self.dead*(llr < 0) - self.dead*(llr > 0)
        lud += self.dead*(lud < 0) - self.dead*(lud > 0)
        rlr += self.dead*(rlr < 0) - self.dead*(rlr > 0)
        rud += self.dead*(rud < 0) - self.dead*(rud > 0)

        button_now = self.js.get_button(3)
        button_event = False
        if button_now and (button_now != self.button_state):
            button_event = True
            
        self.button_state = button_now
        
        return [llr, lud, rlr, rud], button_event
        
if __name__ == "__main__":
    js = JoyStick_helper(0)

    import signal

    running = True
    def exit_handle(signum, frame):
        global running
        running = False
        
    signal.signal(signal.SIGINT, exit_handle)


    while running:
        ax, _, _ = js.get_state()
        print(ax)
