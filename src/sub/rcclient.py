import tkinter as tk
from tkinter import ttk
from typing import List, Tuple
import socket
from threading import Lock
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

            ttk.Label(self.command_frame, text="Target Address:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
            address_options = [a.name for a in Address]
            for row, (cmd, _) in enumerate(self.commands, start=1):
                addr_var = tk.StringVar(value=address_options[1])  # Default MOTION_CTRL
                addr_combo = ttk.Combobox(self.command_frame, textvariable=addr_var, values=address_options, width=15, state="readonly")
                addr_combo.grid(row=row, column=0, padx=5, pady=5, sticky="w")
                self.address_vars[cmd] = addr_var

            for row, (cmd, args) in enumerate(self.commands, start=1):
                ttk.Label(self.command_frame, text=f"{cmd.capitalize()}:").grid(row=row, column=1, padx=5, pady=5, sticky="w")

                col = 2
                self.entries[cmd] = []
                for arg_label, default in args:
                    ttk.Label(self.command_frame, text=arg_label).grid(row=row, column=col, padx=5, pady=5, sticky="w")
                    col += 1
                    if isinstance(default, list):
                        var = tk.StringVar(value=default[0])
                        entry = ttk.Combobox(self.command_frame, textvariable=var, values=default, width=15, state="readonly")
                    else:
                        entry = ttk.Entry(self.command_frame, width=10)
                        entry.insert(0, default)
                    entry.grid(row=row, column=col, padx=5, pady=5)
                    self.entries[cmd].append(entry)
                    col += 1

                send_col = 4 if cmd == "stop" else col
                send_btn = ttk.Button(self.command_frame, text="Send",
                                     command=lambda c=cmd: self.send_command(c))
                send_btn.grid(row=row, column=send_col, padx=5, pady=5)

            self.log_frame = ttk.LabelFrame(self.main_frame, text="Command Log", padding="5")
            self.log_frame.pack(fill="both", expand=True, pady=5)

            self.log_text = tk.Text(self.log_frame, height=20, width=70, state="disabled")
            self.log_text.pack(side="left", fill="both", expand=True)

            scrollbar = ttk.Scrollbar(self.log_frame, orient="vertical", command=self.log_text.yview)
            scrollbar.pack(side="right", fill="y")
            self.log_text.config(yscrollcommand=scrollbar.set)

            self.clear_btn = ttk.Button(self.log_frame, text="Clear Log", command=self.clear_log)
            self.clear_btn.pack(side="bottom", pady=5)

            self.add_to_log(f"Connected to {host}:{port}")

        def send_command(self, command: str) -> None:
            addr = self.address_vars[command].get()
            args = [entry.get() if isinstance(entry, ttk.Entry) else entry.get() for entry in self.entries[command]]
            command_str = f"{addr} {command} {' '.join(args)}".strip()

            try:
                with self.socket_lock:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.connect((self.host, self.port))
                        s.sendall(command_str.encode() + b"\n")
                        response = s.recv(1024).decode().strip()
                self.add_to_log(response)
            except Exception as e:
                self.add_to_log(f"Error: {e}")

        def add_to_log(self, message: str) -> None:
            self.log_text.config(state="normal")
            self.log_text.insert("end", f"{message}\n")
            self.log_text.config(state="disabled")
            self.log_text.see("end")

        def clear_log(self) -> None:
            self.log_text.config(state="normal")
            self.log_text.delete("1.0", tk.END)
            self.log_text.config(state="disabled")

        def run(self) -> None:
            self.root.mainloop()

    client = RemoteControlClient()
    client.run()
