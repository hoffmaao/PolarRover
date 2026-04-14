# Tank mode (two MTTs coupled side-by-side)

In tank mode two MTT-154 units are mechanically coupled and driven as the tracks of a larger vehicle. Each MTT runs its own ECU on its own CAN bus, so the onboard compute needs two independent CAN interfaces. Turning comes from the difference in commanded track velocity rather than from the articulated hitch, so the drive code sends `steer = 0` on each per-side command frame and encodes the turn entirely as asymmetric throttles.

## Block diagram

```mermaid
flowchart LR
    subgraph Sled["Instrument sled enclosure"]
        Jetson["Jetson Orin Nano"]
        GNSS["u-blox ZED-F9P"]
        OLA["OpenLog Artemis"]
        Cam["OAK-D Pro camera"]
        Lidar["D200 LiDAR"]
        DCDC["DC-DC step-down<br/>10–60 V → 5 V / 5 A"]
        USBCAN_L["USB-CAN (left)"]
        USBCAN_R["USB-CAN (right)"]
    end

    subgraph Left["MTT-154 (left track)"]
        BatteryL["Li-ion battery"]
        ECUL["ECU"]
        MotorL["Drive motors"]
    end

    subgraph Right["MTT-154 (right track)"]
        BatteryR["Li-ion battery"]
        ECUR["ECU"]
        MotorR["Drive motors"]
    end

    Antenna["L1/L2 mag-mount antenna"] -->|SMA| GNSS
    GNSS -->|USB| Jetson
    Cam -->|USB| Jetson
    Lidar -->|USB| Jetson
    OLA -.->|UART/USB| Jetson

    Jetson -->|USB| USBCAN_L
    Jetson -->|USB| USBCAN_R
    USBCAN_L -->|CAN_H / CAN_L| ECUL
    USBCAN_R -->|CAN_H / CAN_L| ECUR
    ECUL --> MotorL
    ECUR --> MotorR

    BatteryL -->|auxiliary tap| FuseL["Inline fuse / relay"]
    BatteryR -->|auxiliary tap| FuseR["Inline fuse / relay"]
    FuseL --> DCDC
    FuseR -.->|backup feed| DCDC
    DCDC -->|5 V| Jetson
    DCDC -->|5 V| GNSS
    DCDC -->|5 V| OLA
```

## Connector and harness notes

Each MTT brings its own Deutsch DT 6-pin connector carrying CAN_H, CAN_L, signal ground, and the switched auxiliary power rail. The two connectors land on opposite faces of the enclosure so the left and right harnesses stay separable. Power from the two auxiliary taps is diode-ORed into the DC-DC converter so that the compute keeps running if one unit's battery is swapped mid-mission — but only one bus is load-bearing at a time, so the idle tap sees close to zero current.

Each CAN bus must be independently terminated with 120 Ω at both ends. Mixing the two buses on a single adapter is not supported: the ECUs use the same arbitration IDs for command and feedback, so they must stay on physically separate buses.

## Parts used

Compared to the single-track configuration this adds a second WWZMDiB USB-CAN module (E4 ×2), a second Deutsch DT 6-pin connector (E5), and a second fuse/relay harness (E7). The compute, sensors, DC-DC converter, GNSS, and data logging stay as in the single-track build. See `master_parts_list.xlsx` for quantities.

## Software mapping

The `TankCANBackend` in `rover_hardware.mtt154.tank` wraps two `SingleTrackCANBackend` instances — one for each MTT. When the drive code calls `send(cmd)`, the tank backend runs `mix_differential(cmd, config)` to produce two per-side CommandBus instances and forwards each to its corresponding single-track backend. The mixing math is the same as the simulator's `SideBySideSkidSteer` model, so a scenario that runs cleanly in the emulator produces the same wire commands on hardware.
