# Instructions
UP
```bash
nmcli connection up RobotNet
```

ON LAPTOP 
```bash
ssh team10@10.42.0.1
``` 
(password is the same as the main NUC)

DOWN 
```bash
nmcli connection down RobotNet
```



# For the Pi - need to set up a connection through to wireless:
May also need to edit the Pi's  to export the required ROS_DISCOVERY_SERVER!!

```bash 
nmcli connection show
```

To figure out the `pi-wired-connection-name`. Then add the following route.

```bash
nmcli connection modify <pi-wired-connection-name> \
    ipv4.method manual \
    ipv4.addresses 10.0.0.1/24 \
    ipv4.routes "10.42.0.0/24 10.0.0.20" \
    ipv6.method disabled
```

Then reset the connection 

```bash
nmcli connection down <pi-wired-connection-name>
nmcli connection up <pi-wired-connection-name>
```

Check the routes with `ip addr` and `ip route` and `ip link`


# Discovery servers
To enable the ROS nets to talk to each other, we need to set up a discovery server that will thread ROS messages through the NUC. That looks like each machine exporting the following vars. It is crucial that the ROS_DISCOVERY_SERVER is the ip address of the NUC over the relevant connection (ie. `10.0.0.20`` over ethernet (Pi), or `10.42.0.1` over RobotNet (Laptop) or either on the NUC (localhost `127.0.0.0` should also work)).

On the Pi, instead of exporting these or adding to bashrc they need to launch on startup - modify `/etc/ros/setup.bash` instead.

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="10.0.0.20:11811"
```

The NUC itself needs to run the discovery server:
```bash
fastdds discovery --server-id=0 --udp-port 11811
```


# AI SHITE

# NUC RobotNet Wi-Fi Runbook

This NUC now has an inactive NetworkManager Wi-Fi access point profile called
`RobotNet`.

It has not been started automatically. The current internet Wi-Fi connection is
left active, and the existing wired robot network/bridge is not intentionally
changed.

## Configured State

- Wi-Fi interface: `wlo1`
- NetworkManager profile: `RobotNet`
- SSID: `RobotNet`
- Password: `team10NUC`
- NUC Wi-Fi/AP address: `10.42.0.1/24`
- DHCP/NAT mode: NetworkManager `ipv4.method shared`
- Autoconnect: `no`
- Existing wired robot bridge: left unchanged

The important safety point is that `RobotNet` uses `10.42.0.0/24`, separate
from the existing wired robot subnet.

## Commands Used To Create The Profile

These are the commands that were run on the NUC:

```bash
nmcli connection add type wifi ifname wlo1 con-name RobotNet autoconnect no ssid RobotNet
```

```bash
nmcli connection modify RobotNet 802-11-wireless.mode ap 802-11-wireless.band bg
```

```bash
nmcli connection modify RobotNet ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled
```

```bash
nmcli connection modify RobotNet wifi-sec.key-mgmt wpa-psk wifi-sec.psk team10NUC
```

## Before Starting RobotNet

Do not start `RobotNet` from an SSH session that depends on the NUC's current
Wi-Fi connection. Starting `RobotNet` changes `wlo1` from client mode into
access-point mode, so the NUC will disconnect from normal Wi-Fi/internet.

Prefer to start it when you have one of these:

- physical keyboard/monitor access to the NUC
- Ethernet access to the NUC
- confidence that losing current internet access is acceptable

Check the current state:

```bash
nmcli device status
```

```bash
nmcli connection show RobotNet
```

```bash
ip addr show br0
```

## Start Mission Wi-Fi Mode

Run this on the NUC:

```bash
nmcli connection up RobotNet
```

Expected NUC-side result:

```bash
nmcli device status
```

You should see `wlo1` connected to `RobotNet`.

```bash
ip addr show wlo1
```

You should see `10.42.0.1/24` on `wlo1`.

## Laptop Verification Commands

On the Ubuntu laptop, connect to the Wi-Fi network:

- SSID: `RobotNet`
- Password: `team10NUC`

Then check that the laptop received an address:

```bash
ip addr
```

Look for a Wi-Fi interface with an address like `10.42.0.x`.

Check that the laptop can reach the NUC:

```bash
ping -c 4 10.42.0.1
```

Expected success:

```text
4 packets transmitted, 4 received
```

Test SSH:

```bash
ssh team10@10.42.0.1
```

If SSH fails, try a verbose connection attempt:

```bash
ssh -v team10@10.42.0.1
```

Useful laptop-side failure checks:

```bash
nmcli device status
```

```bash
nmcli connection show --active
```

```bash
ip route
```

The laptop should have a route for `10.42.0.0/24` via its Wi-Fi interface.

## Returning The NUC To Internet Wi-Fi

If the NUC loses internet, that is expected while `RobotNet` is active. The
single Wi-Fi card is being used to host the robot network instead of joining an
internet-providing Wi-Fi network.

On the NUC, stop `RobotNet`:

```bash
nmcli connection down RobotNet
```

Then reconnect to a normal saved Wi-Fi network. For example:

```bash
nmcli connection up "ASK4 Wireless"
```

Or list saved profiles and choose another:

```bash
nmcli connection show
```

```bash
nmcli connection up "Mike's Pixel"
```

Check internet-facing state:

```bash
nmcli device status
```

```bash
ip route
```

You should see `wlo1` connected to a normal Wi-Fi network again, with a default
route through that network.

## Remove RobotNet Completely

Only do this if you want to discard the AP profile:

```bash
nmcli connection down RobotNet
```

```bash
nmcli connection delete RobotNet
```

This removes only the `RobotNet` NetworkManager profile. It should not remove
the existing wired robot bridge or the existing saved internet Wi-Fi profiles.

## Notes About The Raspberry Pi

The first purpose of this setup is reliable laptop-to-NUC SSH over Wi-Fi while
leaving the existing NUC-to-Pi wired network alone.

This does not yet make the laptop part of the Pi's wired subnet. If laptop-to-Pi
ROS visibility is needed later, add that as a separate step after proving that
the Wi-Fi control link works and the existing wired robot network still behaves
normally.
