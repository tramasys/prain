from comms.goal_sound import PWMBuzzer

def test_sound():
    buzzer = PWMBuzzer()
    buzzer.play_goal()
    buzzer.stop()
