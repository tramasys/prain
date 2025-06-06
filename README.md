# prain

The brain of our self driving vehicle ...

## Getting the library

We depend on our `prain_uart` library:

```bash
git clone git@github.com:tramasys/prain_uart_python.git
cd prain_uart_python
pip install -e .
pip show prain_uart # check if installed on machine
```

## Test UART locally (Linux)

Use this to create a non-slaved virtual serial-port character device:

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

And then use the created virtual character devices as the UART port to start the program.

## Set up remote control server

On the PI:

```bash
python src/main.py -s rcserver -u <uart>
```

This sets up a server (host and port can be passed via --host and --port).
The server listens to commands and then writes them to the UART port via execute_command.

On the client:

```bash
python src/main -s rcclient
```

This will open a GUI for you, where you can send commands to the PI (the server), host and port can be passed via --host and --server-port.

## Problems

If you get the following error:

```bash
PermissionError: [Errno 13] Permission denied: '/dev/ttyAMA0'
```

You need to add your current user to the group the character device is assigned to:

```bash
# check groups of user
groups $USER
stat /dev/ttyAMA0
# look at the group defined under Gid:
# add that group to the current user
sudo gpasswd --add $USER $GROUP # or sudo usermod -a -G $GROUP $USER
# logout or reboot
```
