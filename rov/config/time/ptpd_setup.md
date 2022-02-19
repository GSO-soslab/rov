Install `ptpd`
```bash
sudo apt install ptpd
```

Configure ptpd by modifying file `/etc/default/ptpd`
```
# /etc/default/ptpd

# Set to "yes" to actually start ptpd automatically
START_DAEMON="yes"

# Add command line options for ptpd
PTPD_OPTS="-i eth0 -M"
```

Enable systemd service
```bash
sudo systemctl enable ptpd
```

Start ptpd (Not needed after reboot)
```bash
sudo systemctl start ptpd
```

Check if its actually running
```
pidof ptpd
```

purrfect.
