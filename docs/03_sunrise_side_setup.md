# 03 - Install the FRI driver on the KUKA controller (Sunrise side)

Complete, start-to-finish guide for installing the **FRI driver** on the **KUKA LBR Med 7 R800**
controller so it can be driven from ROS 2. It assumes **no prior familiarity** with the robot or this
repo. When you finish, the cabinet runs the **LBRServer** application and accepts FRI from the ROS host.

This is **cabinet-side work only**. It is done from a **Windows** PC running **Sunrise Workbench**; you do
**not** need the ROS host, this repo's code, or `colcon` for any step here. The ROS host is a separate
Linux machine, set up in the other docs (pointers at the end). None of this is needed for mock bringup.

> **Safety first.** Anything under **Safety Configuration** changes how the arm enforces limits, and the
> Med variant carries a medical safety profile. Do **not** invent safety values. Use the lab's documented
> Med safety profile and KUKA's safety manuals. Cell-specific values are marked `[fill in from lab safety config]`.
> On the real robot, always run in **T1** (reduced-speed test) mode first, with the e-stop in hand.

## Glossary (read once before starting)

| Term | What it is |
|---|---|
| **FRI** | Fast Robot Interface: KUKA's real-time UDP control channel between the controller and an external PC. |
| **Cabinet / controller** | The KUKA control box the arm plugs into. Runs **Sunrise.OS Med 1.15**. |
| **Sunrise Workbench** | KUKA's **Windows-only** IDE used to build and install applications onto the cabinet. |
| **smartPAD** | The handheld teach pendant attached to the cabinet. You start the driver from here. |
| **LBRServer** | The Java application that, once installed on the cabinet, speaks FRI to the ROS host. **This is the "FRI driver" you are installing.** |
| **X66 (KLI)** | The cabinet's Ethernet port used here. Default cabinet IP `172.31.1.147`, mask `255.255.0.0` (/16). |
| **`fri`** | The open-source repo (`lbr-stack/fri`) that ships `LBRServer.java`. You clone it during the install. |

## What you need before you start

- A **Windows** PC with **Sunrise Workbench** installed, including the **Sunrise.Servoing** module (required to
  run the FRI server application). Sunrise Workbench is a KUKA download (King's access, see references).
- **Git** and **Windows Terminal** on that PC (both used to fetch `LBRServer.java`).
- One **Ethernet cable** from the PC to the cabinet's **X66** port.
- The cabinet **powered on** and reachable.
- The lab's documented **Med safety profile** and the KUKA **Sunrise.OS Med 1.15** + **Sunrise.FRI 1.15**
  manuals (the authority for safety values and the activation password; see references).

> **Version matching is critical.** The FRI version must match the cabinet OS. This robot ships
> **Sunrise.OS Med 1.15**, which pairs with **FRI 1.15** (`fri` branch `fri-1.15`, ROS-side
> `FRI_CLIENT_VERSION=1.15`, and the repo pin in [`lbr_med_stack.repos`](../lbr_med_stack.repos)).
> If your cabinet shows a different version, see the appendix and use the matching `fri-<major.minor>` branch.

## Step 1 - Confirm the cabinet's Sunrise.OS / FRI version

Confirm the cabinet OS version before installing, so you clone the matching `fri` branch. It is shown in
Sunrise Workbench's Station Setup (and on the smartPAD system info). The lab's robot is expected to be
**Sunrise.OS Med 1.15 -> FRI 1.15**.

> `[VERIFY ON FIRST RUN]` Read the actual version off this cabinet and use it everywhere `1.15` appears
> below. If it is not 1.15, do **not** proceed past Step 6 until the ROS-side `FRI_CLIENT_VERSION` and the
> repo pin are bumped to match (see appendix).

## Step 2 - Connect the Windows PC to the cabinet and verify

1. Turn on the robot (enable the green power switch on the controller).
2. Connect the PC to the controller's **X66** port (default IP `172.31.1.147`) with an Ethernet cable.
3. On the PC, set that NIC to a static address on the same subnet, e.g. `172.31.1.148`, mask `255.255.0.0`.
4. Verify connectivity:

   ```bash
   ping 172.31.1.147
   ```

   Expect replies (`64 bytes from 172.31.1.147: ... time=0.8 ms`). Do not continue until ping succeeds.

> The Windows install PC (`172.31.1.148`) is **not** the ROS host. The ROS host is a separate Linux machine
> that takes `172.31.1.150` on the same X66 subnet (see [`02_network_setup.md`](02_network_setup.md)).

## Step 3 - Create the Sunrise project

In Sunrise Workbench:

1. `File` -> `New` -> `Sunrise project`.
2. Leave the default controller IP `172.31.1.147` and click `Next`.
3. Name the project, e.g. `lbr_fri_ros2`, and click `Next`. (Remember this name; it appears in the path in Step 6.)
4. Select the robot **LBR Med 7 R800** and click `Next`.
5. Select your **Media Flange** type and click `Next`.
6. **Unselect** `Create Sunrise application (starts another wizard)` and click `Finish`.

> `[VERIFY ON FIRST RUN]` The Media Flange type is cell-specific (it is printed on the flange / type plate).
> Pick the one that matches this arm.

## Step 4 - Configure safety

Open `SafetyConfigurations.conf` and set the safety configuration from the lab's documented Med profile and
KUKA's Med safety manuals. Safety changes take effect through the install/reboot in Step 5 (a plain
application sync does not apply safety changes).

- Tool / payload and CoM: `[fill in from lab safety config]`
- Cartesian / workspace monitoring limits: `[fill in from lab safety config]`
- Collaborative speed and force limits: `[fill in from lab safety config]`
- Permitted control modes for FRI: `[fill in from lab safety config]`

Do not proceed to real motion until safety is configured and verified on the smartPAD.

## Step 5 - Configure StationSetup.cat and install to the controller

Open `StationSetup.cat` and:

1. Select your **robot topology**.
2. Select **FRI software and examples** (this adds the FRI extension to the controller image).
3. Configure the network for **X66** and **KONI**.
4. Click `Install` -> `Save and apply`.
5. Click `Ok` to confirm.
6. When asked to reboot, press `OK` and let the controller reboot.
7. After the reboot, **synchronize applications**.

> This installs KUKA's FRI **example** applications (`LBRJointSineOverlay`, `LBRTorqueSineOverlay`,
> `LBRWrenchSineOverlay`). Those alone are **not** enough for this ROS stack: you must also install
> **LBRServer** in Step 6.

## Step 6 - Install the LBRServer application (the FRI driver)

`LBRServer` is the application the ROS stack talks to. Add it to the project:

1. Right-click `src` -> `New` -> `Package`.
2. Name the package `lbr_fri_ros2` and click `Next`.
3. Open **Windows Terminal** and clone the `fri` repo at the branch matching your FRI version
   (replace `1.15` if your cabinet differs):

   ```powershell
   $FRI_CLIENT_VERSION=1.15
   git clone https://github.com/lbr-stack/fri.git -b fri-$FRI_CLIENT_VERSION $HOME\Downloads\fri
   ```

4. Open **Windows Terminal as Administrator** and create a symbolic link to `LBRServer.java` inside the
   package. The path below uses the project and package names from Steps 3 and 6 (`lbr_fri_ros2`); change
   them if you named yours differently:

   ```powershell
   New-Item -ItemType SymbolicLink `
     -Path $HOME\SunriseWorkspace\lbr_fri_ros2\src\lbr_fri_ros2\LBRServer.java `
     -Target $HOME\Downloads\fri\server_app\LBRServer.java
   ```

5. **Refresh** the source in Sunrise Workbench. `LBRServer.java` should now appear under `src`.
6. **Synchronize applications** to push it to the controller.

> After this step the cabinet can run the full LBR FRI ROS 2 Stack.

## Step 7 - Activate execution rights (if the smartPAD blocks you)

Running applications from the smartPAD may require **Safety maintenance technician** rights. If the smartPAD
refuses to start LBRServer:

1. On the smartPAD, navigate to `Safety`.
2. Navigate to `Activation`.
3. Follow `Activation` and enter the password. The password is in section **9.4 User management** of the
   KUKA **Sunrise.OS Med 1.15 / Sunrise.Workbench Med 1.15 Operating and Programming Instructions for
   System Integrators** (see references).

## Step 8 - Start LBRServer on the smartPAD and verify

1. On the smartPAD, **launch the `LBRServer` application**.
2. When prompted, select:
   - **FRI send period:** `10 ms`
   - **FRI control mode:** `POSITION_CONTROL`
   - **FRI client command mode:** `POSITION`
3. LBRServer waits for the ROS host to connect. When the ROS bringup runs (see hand-off below), the smartPAD
   reports the FRI session **connected**.

> Impedance / torque modes (`JOINT_IMPEDANCE_CONTROL`, `CARTESIAN_IMPEDANCE_CONTROL`, command modes
> `TORQUE` / `WRENCH`, and the `2 ms` send period they use) are **future scope**. v1 is **POSITION** only.
> Do not enable them until the impedance path is implemented and safety-reviewed.

The cabinet side is now done. The driver is installed and running.

## Hand-off to the ROS side

The controller is the **server**; the ROS host is the **client**. The FRI driver install (this doc) is
complete on its own, but to see motion you bring up the ROS side on the separate Linux host:

- **Host / port are set on the ROS side, not the smartPAD.** In `lbr_system_config.yaml` (shipped by the
  stack): `port_id: 30200`, `remote_host: INADDR_ANY` (accepts the connection from the cabinet),
  `client_command_mode: position`. FRI is **UDP** on port **30200**.
- ROS host network + connectivity checks: [`02_network_setup.md`](02_network_setup.md).
- Phased mock -> real bringup (start at Phase 0): [`04_hardware_bringup_checklist.md`](04_hardware_bringup_checklist.md).
- First real move at reduced speed: [`examples/goto_pose/README.md`](../examples/goto_pose/README.md).

## Appendix - Cabinet on a different FRI version

If Step 1 showed something other than 1.15:

1. Use the matching `fri-<major.minor>` branch in Step 6. Branches that exist in
   [`lbr-stack/fri`](https://github.com/lbr-stack/fri): `1.11`, `1.13`, `1.14`, `1.15`, `1.16`, `1.17`,
   `2.5`, `2.7`. The branch **must exist** before you clone it.
2. If your version's client SDK is not yet in the `fri` repo, extract it from Sunrise Workbench: in the
   **Software** tab of `StationSetup.cat`, add **Fast Robot Interface Extension**, `Save and apply`, then
   find `FRI-Client-SDK_Cpp.zip` under `FastRobotInterface_Client_Source`, and follow the `fri` repo's
   contributing steps to add a branch.
3. Update the ROS side to match: `FRI_CLIENT_VERSION` ([`01_fresh_machine_setup.md`](01_fresh_machine_setup.md))
   and the pin in [`lbr_med_stack.repos`](../lbr_med_stack.repos).

## References (authoritative sources)

- lbr-stack **Hardware Setup** (the procedure above is grounded in this):
  <https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html>
- lbr-stack **`fri`** repo (ships `LBRServer.java`): <https://github.com/lbr-stack/fri>
- lbr-stack **KUKA documentation** index (links to the KUKA manuals): <https://lbr-stack.readthedocs.io/en/latest/docs/kuka_documentation.html>
- KUKA **Sunrise.FRI 1.15** (proprietary; King's SharePoint / KUKA Xpert).
- KUKA **Sunrise.OS Med 1.15 / Sunrise.Workbench Med 1.15, Operating and Programming Instructions for
  System Integrators** (proprietary; safety configuration + activation password §9.4).
