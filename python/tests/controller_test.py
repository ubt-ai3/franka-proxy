import py_franka_control as pyfranka
import time

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
    print("Initial joint config:")
    print(controller.current_config())

    target = [0.1] * 7
    print("Moving to:", target)
    controller.move(target)
    time.sleep(0.1)
    
    print("New joint config:")
    print(controller.current_config())

    # Gripper test
    controller.open_gripper()
    time.sleep(0.1)
    print("Gripper opened:", not controller.gripper_grasped())

    controller.close_gripper()
    time.sleep(0.1)
    print("Gripper closed:", controller.gripper_grasped())
    
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