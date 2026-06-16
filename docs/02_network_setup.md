# 02 - Network setup (FRI subnet)

Cable the host to the cabinet (directly or through the lab switch) and give the host a static address on the
FRI subnet. Do this before any real-robot bringup. None of this is needed for mock bringup.

## KUKA interface map

The cabinet exposes two relevant Ethernet ports:


| Port    | Name                                 | Default cabinet IP | Mask                  | Used by this stack |
| ------- | ------------------------------------ | ------------------ | --------------------- | ------------------ |
| **X66** | KLI (KUKA Line Interface)            | `172.31.1.147`     | `255.255.0.0` (/16)   | **Yes**            |
| X69     | KONI (KUKA Option Network Interface) | `192.170.10.2`     | `255.255.255.0` (/24) | No                 |


lbr-stack's documented flow drives FRI over **X66 (KLI)**. We cable into **X66**, not KONI. Revisit only if
the lab's cabinet turns out to be wired for KONI.

## Static IP config

If the host reaches the cabinet through an Ethernet switch shared with the rest of the lab network, add a
static address for each robot to that NIC on top of its DHCP lease. This is how.

Find the interface name first (it should look like `enp<X>s<Y>`):

```bash
ip link
```

Then edit `/etc/netplan/01-netcfg.yaml` (`sudo nano /etc/netplan/01-netcfg.yaml`), replacing `enp<X>s<Y>`
with the real interface name:

```yaml
# /etc/netplan/01-netcfg.yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    enp130s0:
      dhcp4: true            # gets default route + DNS from the lab network
      optional: true
      addresses:
        - 192.168.1.20/24    # kinova gen 3
        - 172.31.1.150/16    # kuka lbr med 7 r800 (X66/KLI)
      nameservers:
        addresses: [8.8.8.8] # harmless backup; DHCP DNS takes priority
```

Apply and verify:

```bash
sudo netplan generate && sudo netplan apply
ip addr show enp<X>s<Y>
```

> This is the shared lab NIC, so DHCP (default route + DNS) stays on; the two static `addresses:` ride on
> top of it, one per robot. Keep the KUKA address on **/16** (next paragraph), and make sure neither static
> subnet (`192.168.1.0/24`, `172.31.0.0/16`) overlaps another network reachable through the switch.

The mask is **/16 (255.255.0.0)**, the KUKA X66 default, not /24. A /24 still reaches `172.31.1.147`, but /16 is
the KUKA-correct value and avoids surprises if the cabinet hands out addresses outside the `.1.x` block.

## FRI transport

- Protocol: **UDP**.
- Port: **30200** (FRI default).
- Send period: **10 ms** (matches `FRI_CLIENT_VERSION=1.15`).

## Connectivity checks

Once cabled and netplan applied (cabinet powered, but before starting FRI on the smartPAD):

```bash
ping -c4 172.31.1.147     # reach the cabinet KLI
ip neigh                  # confirm the cabinet MAC shows up in the ARP table
```

After the FRI server (LBRServer) is running on the cabinet and the ROS overlay is up, confirm the FRI handshake
from the ROS side:

- LBRServer reports **connected** on the smartPAD.
- The `lbr/state` topics publish:

```bash
ros2 topic list | grep lbr
ros2 topic echo /lbr/state --once
```

> [VERIFY ON FIRST RUN] The exact state topic name(s) and namespace (`/lbr/...`) come from `lbr_bringup`'s
> default remappings on the pinned tag. Confirm with `ros2 topic list` once bringup is running.

## Next

Cabinet-side setup is in [`03_sunrise_side_setup.md`](03_sunrise_side_setup.md).