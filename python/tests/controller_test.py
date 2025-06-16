import py_franka_control as pyfranka
import numpy as np
import time


def test_remote_controller():
    print("Testing remote python franka controller:")
    robot_ip = "127.0.0.1"
    controller = pyfranka.FrankaControllerRemote(robot_ip)
    update_task = pyfranka.FrankaUpdateTask(controller)
    time.sleep(1)

    # Motion test
    print("Init config:\t", controller.current_config())
    
    start = np.array([-1.07627182, -0.34426202, 0.58798929, -2.32064995, 0.30559797, 2.0511126, 0.31874809])
    print("Moving to:\t", start)
    controller.move(start)
    time.sleep(0.02)
    
    print("Start config:\t", controller.current_config())
    
    target = np.array([-0.60258097, -0.43091224, 0.65565041, -2.32016194, 0.3065638, 2.03325975, 0.56172062])
    print("Moving to:\t", target)
    controller.move(target)
    time.sleep(0.02)

    print("End config:\t", controller.current_config())

    # Gripper test
    controller.close_gripper()
    time.sleep(0.02)
    print("Gripper closed.")
    
    controller.open_gripper()
    time.sleep(0.02)
    print("Gripper opened.")
    
    # Status test: Hand guide the robot, update_task should handle the franka config updates
    for i in range(100):
        print("Config:", controller.current_config())
        time.sleep(0.1)
    
    
def test_emulated_controller():
    target = np.array([-0.60258097, -0.43091224, 0.65565041, -2.32016194, 0.3065638, 2.03325975, 0.56172062])

    # Testing fk-ik 
    print("Target config: ", target)
    fk_array = pyfranka.FrankaProxyUtil.fk(target)
    print("First fk result: ", fk_array[-1])
    ik_result = pyfranka.FrankaProxyUtil.ik_fast_closest(fk_array[-1], target)
    print("Resulting ik config: ", ik_result)
    fk_array = pyfranka.FrankaProxyUtil.fk(ik_result)
    print("Second fk result: ", fk_array[-1])
  
    print("Testing emulated python franka controller:")
    controller = pyfranka.FrankaControllerEmulated()

    print("Initial joint config:", controller.current_config())
    
    print("Emulated move to:", target)
    controller.move(target)
    # No need to controller.update() for emulated use.
    print("New joint config:", controller.current_config())



if __name__ == "__main__":
    np.set_printoptions(floatmode='fixed', formatter={'float': '{:0.4f}'.format})
        
    #test_remote_controller()
    test_emulated_controller()