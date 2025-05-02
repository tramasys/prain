from prain_uart import *

def frame_debug(frame: Frame) -> None:
    decoder = Decoder(frame)

    print("==================== Frame Debug ====================")
    print(f"Frame (hex): {frame.raw:016X}")
    print(f"Frame (bin): {frame.raw:064b}")
    print(f"Address: {decoder.address.name} ({decoder.address.value})")
    print(f"Command: {decoder.command.name} ({decoder.command.value})")
    print(f"CRC valid: {decoder.verify_crc()}")

    try:
        params = decoder.get_params()
        if isinstance(params, MoveParams):
            print(f"Params: distance={params.distance}")
        elif isinstance(params, ReverseParams):
            print(f"Params: distance={params.distance}")
        elif isinstance(params, TurnParams):
            print(f"Params: angle={params.angle}")
        elif isinstance(params, InfoParams):
            print(f"Params: flag={_safe_enum_name(InfoFlag, params.flag)}")
        elif isinstance(params, EmptyParams):
            print("Params: None (Empty)")
        elif isinstance(params, ErrorParams):
            print(f"Params: code={_safe_enum_name(ErrorCode, params.error_code)}")
        elif isinstance(params, PingParams):
            print(f"Params: id={params.id}")
        elif isinstance(params, PongParams):
            print(f"Params: id={params.id}")
        elif isinstance(params, PollParams):
            print(f"Params: id={_safe_enum_name(PollId, params.poll_id)}")
        elif isinstance(params, ResponseParams):
            print(f"Params: id={params.poll_id}, data={params.data}")
        else:
            print("Params: Unknown type!")
    except ValueError as e:
        print(f"Error decoding params: {e}!")

    print("========================================================")

def _safe_enum_name(enum_cls, value):
    try:
        return f"{enum_cls(value).name} ({value})"
    except ValueError:
        return f"UNKNOWN ({value})"
