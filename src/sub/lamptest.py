from comms.lamp import GPIOLamp

def test_lamp():
    lamp = GPIOLamp()
    print("Turning lamp on ...")
    lamp.on()
    print("Lamp is on!")
    input("Press Enter to turn off the lamp ...")
    lamp.off()
    print("Lamp is off!")
    lamp.cleanup()
