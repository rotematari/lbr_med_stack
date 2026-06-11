# 02 - Network setup (FRI subnet)

Cable the host to the cabinet and put a dedicated NIC on the FRI subnet. Do this before any real-robot bringup.
None of this is needed for mock bringup.

## KUKA interface map

The cabinet exposes two relevant Ethernet ports:

| Port | Name | Default cabinet IP | Mask | Used by this stack |
|---|---|---|---|---|
| **X66** | KLI (KUKA Line Interface) | `172.31.1.147` | `255.255.0.0` (/16) | **Yes** |
| X69 | KONI (KUKA Option Network Interface) | `192.170.10.2` | `255.255.255.0` (/24) | No |

lbr-stack's documented flow drives FRI over **X66 (KLI)**. We cable into **X66**, not KONI. Revisit only if
the lab's cabinet turns out to be wired for KONI.

## NIC config (netplan)

Use a **dedicated** NIC for the FRI link, separate from your internet NIC. The host takes a static address on
the KLI subnet, with **no gateway and no DNS** on this link, so your default internet route is untouched.

Find the interface name first (it is **not** your main/internet NIC):

```bash
ip link
```

Then drop in `config/netplan/99-fri-x66.yaml`, replacing `enpXsY` with the real interface name:

```yaml
# /etc/netplan/99-fri-x66.yaml
# Dedicated NIC for the KUKA LBR Med X66 (KLI) FRI subnet.
# Host static IP, NO gateway and NO DNS on this link, so default internet routing is untouched.
# Replace enpXsY with the real interface name from `ip link` (NOT your main/internet NIC).
network:
  version: 2
  renderer: networkd
  ethernets:
    enpXsY:                       # <-- the dedicated FRI NIC (find via `ip link`)
      dhcp4: false
      dhcp6: false
      addresses:
        - 172.31.1.150/16         # host on the KUKA KLI subnet (mask 255.255.0.0 - KUKA default)
      # no gateway4/routes/nameservers: this link carries only FRI traffic to the cabinet
```

Apply and verify:

```bash
sudo cp config/netplan/99-fri-x66.yaml /etc/netplan/99-fri-x66.yaml
sudo chmod 600 /etc/netplan/99-fri-x66.yaml
sudo netplan generate && sudo netplan apply
ip addr show enpXsY
```

> **Do not** put this address on your internet NIC, and do not add a `gateway4`, `routes:`, or
> `nameservers:` block. The link carries only FRI traffic; adding a default route here can hijack your
> internet routing.

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
