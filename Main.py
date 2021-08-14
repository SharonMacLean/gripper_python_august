from tkinter import *
import time


# Import Gripper and GripperGUI classes
from Gripper import Gripper
from GripperGUI import GripperGUI
from ControlScheme import PythonController

kukaGripperController = PythonController()
kukaGripper1 = Gripper(controller=kukaGripperController, fingertype="Rigid")

root = Tk()
kukaGripperGUI = GripperGUI(root, kukaGripper1)
numDecimals = 3  # Number of decimals to show when displaying the gripper measurements

kukaGripper1.initialize_gripper(serial_port='/dev/ttyACM0', baud=115200, serial_timeout=0.01, gui=kukaGripperGUI)


def main():
    #dt = [0 for _ in range(1000)]
    #for i in range(1000):
    #    t1 = time.time()
    #    kukaGripper1.gripper_loop()
    #    t2 = time.time()
    #    dt[i] = t2-t1
    #print(dt)
    #input(sum(dt)/1000)
    while 1:
        try:
            mainloop()
        except:
            print("Closing communications")
            kukaGripper1.close_communications()
            break

# Main loop of the program. The measurement values shown in the Gripper GUI window are updated here.
def mainloop():
    kukaGripper1.gripper_loop()

    # Update the current gripper force displayed by the GUI
    currentforce = kukaGripper1.currentforce
    if currentforce is None:
        kukaGripperGUI.curforcevar.set("X N")
    # If the current position is a number
    else:
        kukaGripperGUI.curforcevar.set(("{:." + str(numDecimals) + "f}").format(currentforce) + " N")


    # Update the current position of the gripper displayed by the GUI
    currentpositiontrue = kukaGripper1.currentpositiontrue
    if currentpositiontrue is None:
        kukaGripperGUI.cursizevar.set("X mm")
    # If the current position is a number
    else:
        kukaGripperGUI.cursizevar.set(("{:." + str(numDecimals) + "f}").format(currentpositiontrue) + " mm")


    # Update the current position uncertainty of the gripper displayed by the GUI
    kukaGripperGUI.posuncertaintyvar.set(("{:." + str(numDecimals) + "f}").format(
        kukaGripper1.currentpositionuncertainty) + " mm")

    # Update the gripper status displayed by the GUI
    kukaGripperGUI.curstatevar.set(kukaGripper1.status)

    # Update the GUI window
    root.update()

if __name__ == '__main__':
    main()