sudo cp ./80-can.network /lib/systemd/network/
sudo systemctl start systemd-networkd.service
sudo systemctl enable systemd-networkd.service