add user `gpsd` into `tty` group.
```bash
sudo adduser gpsd tty
```

Disable geTTY server on ttyTHS0
```bash
sudo systemctl stop nvgetty
```

