import py_franka_control as pyfranka
import time
import threading


class UpdateThread:
    def __init__(self, controller, interval_sec=0.02):
        self.controller = controller
        self.interval = interval_sec
        self._running = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()

    def _run(self):
        while self._running:
            try:
                self.controller.update()
            except Exception as e:
                print("Update error:", e)
            time.sleep(self.interval)


def test_remote_controller():
    print("Testing remote python franka controller:")
    robot_ip = "127.0.0.1"
    controller = pyfranka.FrankaControllerRemote(robot_ip)
 
    updater = UpdateThread(controller)
    updater.start()
 
    # Motion test
    print("Initial joint config:", controller.current_config())
    
    start = [-1.07627182, -0.34426202, 0.58798929, -2.32064995, 0.30559797, 2.0511126, 0.31874809]
    print("Moving to:", start)
    controller.move(start)
    time.sleep(0.1)
    
    target = [-0.60258097, -0.43091224, 0.65565041, -2.32016194, 0.3065638, 2.03325975, 0.56172062]
    print("Moving to:", target)
    controller.move(target)
    time.sleep(0.1)
    
    print("End joint config:", controller.current_config())

    # Gripper test
    controller.close_gripper()
    time.sleep(0.1)
    print("Gripper closed.")
    
    controller.open_gripper()
    time.sleep(0.1)
    print("Gripper opened.")

    updater.stop()
    
    
def test_emulated_controller():
    print("Testing emulated python franka controller:")
    controller = pyfranka.FrankaControllerEmulated()

    print("Initial joint config:")
    print(controller.current_config())

    target = [0.5] * 7
    print("Moving to:", target)
    controller.move(target)

    # No need to controller.update() for emulated use.

    print("New joint config:")
    print(controller.current_config())


if __name__ == "__main__":
    #test_remote_controller()
    test_emulated_controller()