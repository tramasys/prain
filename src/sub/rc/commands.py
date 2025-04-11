from comms.manager import UartManager
from prain_uart import *

def execute_command(uart_mgr: UartManager, command_str: str) -> str:
    print(f"Received command: {command_str}")
    parts = command_str.strip().lower().split()
    if len(parts) < 2:
        return "Error: Command must include address"

    addr_str, cmd = parts[0], parts[1]
    try:
        addr = Address[addr_str.upper()]
    except KeyError:
        return f"Error: Invalid address: {addr_str}"

    try:
        if cmd == "move" and len(parts) == 3:
            distance = int(parts[2])
            uart_mgr.send_frame(encode_move(addr, distance))
            return f"{addr.name} MOVE {distance} cm"
        elif cmd == "reverse" and len(parts) == 3:
            distance = int(parts[2])
            uart_mgr.send_frame(encode_reverse(addr, distance))
            return f"{addr.name} REVERSE {distance} cm"
        elif cmd == "turn" and len(parts) == 3:
            angle = int(parts[2])
            uart_mgr.send_frame(encode_turn(addr, angle))
            return f"{addr.name} TURN {angle} degrees"
        elif cmd == "stop" and len(parts) == 2:
            uart_mgr.send_frame(encode_stop(addr))
            return f"{addr.name} STOP"
        elif cmd == "info" and len(parts) == 3:
            flag = InfoFlag[parts[2].upper()].value
            uart_mgr.send_frame(encode_info(addr, flag))
            return f"{addr.name} INFO flag={flag}"
        elif cmd == "ping" and len(parts) == 3:
            id = int(parts[2])
            uart_mgr.send_frame(encode_ping(addr, id))
            return f"{addr.name} PING id={id}"
        elif cmd == "pong" and len(parts) == 3:
            id = int(parts[2])
            uart_mgr.send_frame(encode_pong(addr, id))
            return f"{addr.name} PONG id={id}"
        elif cmd == "error" and len(parts) == 3:
            error_code = int(parts[2])
            uart_mgr.send_frame(encode_error(addr, error_code))
            return f"{addr.name} ERROR code={error_code}"
        elif cmd == "poll" and len(parts) == 3:
            poll_id = PollId[parts[2].upper()].value
            uart_mgr.send_frame(encode_poll(addr, poll_id))
            return f"{addr.name} POLL id={poll_id}"
        elif cmd == "response" and len(parts) == 4:
            poll_id = PollId[parts[2].upper()].value
            data = int(parts[3])
            uart_mgr.send_frame(encode_response(addr, poll_id, data))
            return f"{addr.name} RESPONSE poll_id={poll_id}, data={data}"
        elif cmd == "grip" and len(parts) == 2:
            uart_mgr.send_frame(encode_grip(addr))
            return f"{addr.name} GRIP"
        elif cmd == "release" and len(parts) == 2:
            uart_mgr.send_frame(encode_release(addr))
            return f"{addr.name} RELEASE"
        else:
            return f"Error: Unknown command or invalid arguments: {command_str}"
    except (KeyError, ValueError) as e:
        return f"Error: Invalid parameter: {e}"
