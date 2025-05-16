from comms.go_button import GoButton

def test_go_button():
    go_button = GoButton(active_high=True)
    print("Press the button to test...")
    go_button.wait_for_press()
    print("Button pressed!")
    go_button.cleanup()
