# 03 - Sunrise side setup (cabinet)

Cabinet-side work to make the arm accept FRI from the ROS host. This runs in **Sunrise Workbench** on a
Windows machine talking to the cabinet over X66. None of this is needed for mock bringup.

> Safety first. Anything that touches Safety Configuration changes how the arm enforces limits. Do not
> invent safety values. Use the lab's documented Med safety profile and KUKA's safety manuals. Cell-specific
> values below are marked `[fill in from lab safety config]`.

## Prerequisites

- **Sunrise Workbench** installed, with the **Sunrise.Servoing** module (required by lbr-stack to run the FRI
  server application).
- **Sunrise.OS Med 1.15** on the cabinet (pairs with `FRI_CLIENT_VERSION=1.15`).
- Workbench host on the X66 subnet, able to reach the cabinet at `172.31.1.147`.

## 1. Load / sync the project from the controller

1. In Sunrise Workbench, connect to the controller at `172.31.1.147`.
2. Open **Station Setup -> Installation -> Synchronize** to pull the current project from the cabinet.
3. Note: changing **Safety Configuration** requires a reinstall (a plain sync does not apply safety changes).

## 2. Install the FRI server application (LBRServer)

lbr-stack ships a Sunrise application, **LBRServer**, that runs the FRI server on the cabinet.

1. Add the LBRServer application from `lbr_fri_ros2_stack`'s Sunrise app sources to the Workbench project.
2. Build the project in Workbench.
3. Deploy / synchronize to the cabinet (**Installation -> Synchronize**).

> [VERIFY ON FIRST RUN] The exact LBRServer source location and Workbench import steps come from lbr-stack's
> KUKA-side documentation for the pinned tag. Confirm against
> <https://lbr-stack.readthedocs.io/en/latest/docs/kuka_documentation.html> before deploying.

## 3. Configure LBRServer at runtime (smartPAD)

Start the LBRServer application from the smartPAD and set:

- **FRI send period:** 10 ms.
- **Client IP (host):** `172.31.1.150` (the ROS host on the FRI subnet).
- **Port:** 30200.
- **Control mode:** **POSITION** for v1.

> Impedance control (`JOINT_IMPEDANCE`) is **future scope**, not v1. Do not enable it until the impedance path
> is implemented and safety-reviewed.

## 4. Safety configuration (Med variant)

The Med variant carries a medical safety profile. Set the safety configuration from the lab's documented
profile and KUKA's Med safety manuals, then reinstall (per step 1, safety changes require reinstall).

- Tool / payload and CoM: `[fill in from lab safety config]`
- Cartesian / workspace monitoring limits: `[fill in from lab safety config]`
- Collaborative speed and force limits: `[fill in from lab safety config]`
- Permitted control modes for FRI: `[fill in from lab safety config]`

Do not proceed to real bringup until safety is configured and verified on the smartPAD.

## Next

Phased bringup (mock then real) is in [`04_hardware_bringup_checklist.md`](04_hardware_bringup_checklist.md).
