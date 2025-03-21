# prain

The brain of our self driving vehicle ...

## Test UART locally
```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
And then use the created virtual character devices as the port to start the program.

## Set up remote control server
On the PI:
```bash
python src/main.py -s rcserver -p <uart port>
```

On the client:
```bash
python src/main -s rcclient
```

This opens a window, where you can send commands to the PI.

## Problems
If you get the following error:
```bash
PermissionError: [Errno 13] Permission denied: '/dev/ttyS0'
```

You need to add your current user to the group the character device is assigned to:
```bash
# check groups of user
groups $USER
stat /dev/ttyS0
# look at the group defined under Gid:
# add that group to the current user
sudo gpasswd --add $USER $GROUP # or sudo usermod -a -G $GROUP $USER
# logout or reboot
```
