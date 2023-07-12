from sshkeyboard import listen_keyboard
import threading
import time

control_period = 0.02

def press(key):
    if key == 'a':
        print("a was pressed")



thread = threading.Thread(target=lambda: listen_keyboard(on_press=press, sequential=True))
thread.start()

while True:
    start_time = time.time_ns()

    end_time = time.time_ns()
    
    print("loop rate: ", 1/((end_time - start_time)/1e9))
    time.sleep(max(0, control_period - (end_time - start_time)/1e9))
