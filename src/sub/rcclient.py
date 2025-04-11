import tkinter as tk
from tkinter import ttk
import socket
from threading import Lock
from typing import List, Tuple
from prain_uart import Address, InfoFlag, CraneFlag, PollId

def rc_client(host: str = "localhost", port: int = 5000) -> None:
    class RemoteControlClient:
        def __init__(self):
            self.host = host
            self.port = port
            self.socket_lock = Lock()

            self.root = tk.Tk()
            self.root.title(f"Remote Control Client ({host}:{port})")
            self.root.geometry("850x800")

            self.commands: List[Tuple[str, List[Tuple[str, str | list]]]] = [
                ("move", [("Distance (cm)", "0")]),
                ("reverse", [("Distance (cm)", "0")]),
                ("turn", [("Angle (degrees)", "0")]),
                ("stop", []),
                ("info", [("Flag", [f.name for f in InfoFlag])]),
                ("ping", [("ID (0-255)", "0")]),
                ("pong", [("ID (0-255)", "0")]),
                ("error", [("Error Code (0-65535)", "0")]),
                ("poll", [("Poll ID", [p.name for p in PollId])]),
                ("response", [("Poll ID", [p.name for p in PollId]), ("Data (0-65535)", "0")]),
                ("crane", [("Flag", [c.name for c in CraneFlag])]),
            ]

            self.main_frame = ttk.Frame(self.root, padding="10")
            self.main_frame.pack(fill="both", expand=True)

            self.command_frame = ttk.LabelFrame(self.main_frame, text="Commands", padding="5")
            self.command_frame.pack(fill="x", pady=5)

            self.entries = {}
            self.address_vars = {}

            address_options = [a.name for a in Address]
            for row, (cmd, arglist) in enumerate(self.commands, start=1):
                addr_var = tk.StringVar(value=address_options[1])  # default to MOTION_CTRL (if it exists)
                addr_combo = ttk.Combobox(
                    self.command_frame, textvariable=addr_var,
                    values=address_options, width=12, state="readonly"
                )
                addr_combo.grid(row=row, column=0, padx=5, pady=5, sticky="w")
                self.address_vars[cmd] = addr_var

                ttk.Label(self.command_frame, text=f"{cmd.capitalize()}:").grid(
                    row=row, column=1, padx=5, pady=5, sticky="w"
                )

                col = 2
                self.entries[cmd] = []
                for arg_label, default in arglist:
                    ttk.Label(self.command_frame, text=arg_label).grid(
                        row=row, column=col, padx=5, pady=5, sticky="w"
                    )
                    col += 1

                    if isinstance(default, list):
                        # pick first option as default
                        var = tk.StringVar(value=default[0])
                        entry = ttk.Combobox(self.command_frame, textvariable=var, values=default, width=12, state="readonly")
                    else:
                        entry = ttk.Entry(self.command_frame, width=8)
                        entry.insert(0, default)
                    entry.grid(row=row, column=col, padx=5, pady=5)
                    self.entries[cmd].append(entry)
                    col += 1

                send_btn = ttk.Button(
                    self.command_frame, text="Send",
                    command=lambda c=cmd: self.send_command(c)
                )
                send_btn.grid(row=row, column=col, padx=5, pady=5)

            # Now create two vertical log panes
            self.logs_frame = ttk.Frame(self.main_frame)
            self.logs_frame.pack(fill="both", expand=True)

            # Left side -> "Send Commands"
            self.send_log_frame = ttk.LabelFrame(self.logs_frame, text="Sent commands", padding="5")
            self.send_log_frame.pack(side="left", fill="both", expand=True)

            self.send_log_text = tk.Text(self.send_log_frame, height=20, width=50, state="disabled")
            self.send_log_text.pack(side="left", fill="both", expand=True)

            send_scrollbar = ttk.Scrollbar(self.send_log_frame, orient="vertical", command=self.send_log_text.yview)
            send_scrollbar.pack(side="right", fill="y")
            self.send_log_text.config(yscrollcommand=send_scrollbar.set)

            # Right side -> "Received Commands"
            self.recv_log_frame = ttk.LabelFrame(self.logs_frame, text="Received commands", padding="5")
            self.recv_log_frame.pack(side="left", fill="both", expand=True)

            self.recv_log_text = tk.Text(self.recv_log_frame, height=20, width=50, state="disabled")
            self.recv_log_text.pack(side="left", fill="both", expand=True)

            recv_scrollbar = ttk.Scrollbar(self.recv_log_frame, orient="vertical", command=self.recv_log_text.yview)
            recv_scrollbar.pack(side="right", fill="y")
            self.recv_log_text.config(yscrollcommand=recv_scrollbar.set)

        def send_command(self, cmd_name: str):
            addr = self.address_vars[cmd_name].get()
            arg_values = [entry.get() for entry in self.entries[cmd_name]]
            cmd_str = f"{addr} {cmd_name} {' '.join(arg_values)}".strip()

            response = "<No response>"
            try:
                with self.socket_lock:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.connect((self.host, self.port))
                        s.sendall(cmd_str.encode() + b"\n")
                        response = s.recv(1024).decode().strip()
            except Exception as e:
                response = f"Error: {e}"

            self.add_to_send_log(cmd_str)
            # self.add_to_send_log(f"RESP < {response}")

        def poll_received(self):
            """Continuously poll for new messages from the server."""
            try:
                with self.socket_lock:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.connect((self.host, self.port))
                        s.sendall(b"get_received\n")
                        response = s.recv(4096).decode().strip()

                if response and response != "":
                    self.add_to_recv_log(response)
            except Exception as e:
                self.add_to_recv_log(f"Error polling: {e}")

            self.root.after(500, self.poll_received)

        def add_to_send_log(self, message: str):
            self.send_log_text.config(state="normal")
            self.send_log_text.insert("end", f"{message}\n")
            self.send_log_text.config(state="disabled")
            self.send_log_text.see("end")

        def add_to_recv_log(self, message: str):
            if not message.strip():
                return

            self.recv_log_text.config(state="normal")
            self.recv_log_text.insert("end", f"{message}\n")
            self.recv_log_text.config(state="disabled")
            self.recv_log_text.see("end")

        def run(self):
            self.root.after(1000, self.poll_received)
            self.root.mainloop()

    RemoteControlClient().run()
