from comms.goal_sound import PWMBuzzer

def test_sound():
    buzzer = PWMBuzzer()
    print("Playing goal sound...")
    buzzer.play_rickroll()
    print("Sound played!")
    buzzer.stop()
