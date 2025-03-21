# prain

The brain of our self driving vehicle ...

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
