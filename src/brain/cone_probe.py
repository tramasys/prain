class ConeProbeSweep:
    def __init__(self, base_angle: int, angle_offsets: list[int]):
        self.base_angle = base_angle
        self.angle_offsets = angle_offsets
        self.current_index = 0
        self.total_rotation = 0
        self.active = True

    def get_current_offset(self) -> int:
        return self.angle_offsets[self.current_index]

    def advance(self) -> bool:
        self.current_index += 1
        return self.current_index < len(self.angle_offsets)

    def get_next_turn_amount(self) -> int:
        if self.current_index == 0:
            return self.angle_offsets[0]
        return self.angle_offsets[self.current_index] - self.angle_offsets[self.current_index - 1]

    def record_rotation(self, delta: int):
        self.total_rotation += delta

    def reset(self):
        self.current_index = 0
        self.total_rotation = 0
        self.active = True

    def stop(self):
        self.active = False

    def is_done(self) -> bool:
        return self.current_index >= len(self.angle_offsets)
