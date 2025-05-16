from comms.target import TargetDetector

def target_switch_test():
    detector = TargetDetector()
    target   = detector.detect()
    print(f"Selected target: {target}")
    detector.cleanup()
