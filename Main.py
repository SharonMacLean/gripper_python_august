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
            kukaGripper1.close_communications()
            break

def mainloop():
    kukaGripper1.gripper_loop()
    kukaGripperGUI.curforcevar.set(kukaGripper1.currentforce)
    kukaGripperGUI.cursizevar.set(kukaGripper1.currentpositiontrue)
    kukaGripperGUI.posuncertaintyvar.set(kukaGripper1.currentpositionuncertainty)
    kukaGripperGUI.curstatevar.set(kukaGripper1.status)
    root.update()

if __name__ == '__main__':
    main()