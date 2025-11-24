# TCP Communication Architecture

## System Overview
Communication between Apple Vision Pro (Swift) and ROS2 System (Python) via TCP on 3 ports.

---

## 1. Mermaid Diagram

```mermaid
graph TB
    subgraph AVP["🍎 Apple Vision Pro (Swift)"]
        UI[User Interface]
        TC1[TCPClient :5000]
        TC2[DrillPosesTCPClient :5001]
        TC3[ImageTCPClient :5002]
    end
    
    subgraph ROS["🤖 ROS2 System (Python/Linux)"]
        TCP[tcp_server_node.py]
        CTRL[Control Logic]
        POSES[Drill Trajectories]
        SEG[Segmentation Service]
    end
    
    UI --> TC1
    UI --> TC2
    UI --> TC3
    
    TC1 <-->|":5000<br/>One-Time Control Commands<br/>(PROCEED, RESET, KILLALL)"| TCP
    TC2 <-->|":5001<br/>Large Data Transfer<br/>(Drill Site Poses/Trajectories)"| TCP
    TC3 <-->|":5002<br/>Continuous Streaming<br/>(Images + Annotations)"| TCP
    
    TCP --> CTRL
    TCP --> POSES
    TCP --> SEG
    
    style AVP fill:#ff9966,stroke:#333,stroke-width:3px
    style ROS fill:#6699ff,stroke:#333,stroke-width:3px
    style TC1 fill:#ffcc99,stroke:#333,stroke-width:2px
    style TC2 fill:#ffcc99,stroke:#333,stroke-width:2px
    style TC3 fill:#ffcc99,stroke:#333,stroke-width:2px
```

---

## 2. ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     TCP Communication Architecture                       │
└─────────────────────────────────────────────────────────────────────────┘

    ┌───────────────────────┐                    ┌──────────────────────┐
    │   Apple Vision Pro    │                    │    ROS2 System       │
    │      (Swift App)      │                    │   (Python/Linux)     │
    │                       │                    │                      │
    │  ┌─────────────────┐  │                    │  ┌────────────────┐  │
    │  │  UI Controls    │  │                    │  │ tcp_server_node│  │
    │  └────────┬────────┘  │                    │  └────────┬───────┘  │
    │           │           │                    │           │          │
    │  ┌────────▼────────┐  │                    │  ┌────────▼───────┐  │
    │  │ TCPClient :5000 ├──┼────────────────────┼─►│  Port 5000     │  │
    │  │  (Control)      │  │  One-Time Commands │  │  (Commands)    │  │
    │  └─────────────────┘  │  PROCEED, RESET,   │  └────────────────┘  │
    │                       │  KILLALL, etc.     │                      │
    │  ┌─────────────────┐  │                    │  ┌────────────────┐  │
    │  │DrillPosesTCP    │◄─┼────────────────────┼──┤  Port 5001     │  │
    │  │Client :5001     │  │  Large Data Xfer   │  │ (Drill Poses)  │  │
    │  └─────────────────┘  │  Trajectories/Poses│  └────────────────┘  │
    │                       │                    │                      │
    │  ┌─────────────────┐  │                    │  ┌────────────────┐  │
    │  │ ImageTCPClient  │◄─┼────────────────────┼─►│  Port 5002     │  │
    │  │   :5002         │  │  Continuous Stream │  │  (Images)      │  │
    │  └─────────────────┘  │  Images + Annot.   │  └────────────────┘  │
    │                       │                    │                      │
    └───────────────────────┘                    └──────────────────────┘
    
    192.168.0.193 (ROS PC IP Address)
```

---

## 3. PlantUML Diagram

```plantuml
@startuml
!define RECTANGLE class

skinparam rectangle {
    BackgroundColor<<AVP>> #FF9966
    BackgroundColor<<ROS>> #6699FF
    BackgroundColor<<Port>> #FFCC99
    BorderColor #333333
    BorderThickness 2
}

rectangle "Apple Vision Pro\n(Swift App)" <<AVP>> as AVP {
    rectangle "TCPClient" <<Port>> as TC1
    rectangle "DrillPosesTCPClient" <<Port>> as TC2
    rectangle "ImageTCPClient" <<Port>> as TC3
}

rectangle "ROS2 System\n(Python/Linux PC)" <<ROS>> as ROS {
    rectangle "tcp_server_node.py" as SERVER
    rectangle "Control Logic" as CTRL
    rectangle "Drill Trajectories" as TRAJ
    rectangle "Segmentation" as SEG
}

TC1 -right-> SERVER : **:5000**\nOne-Time Control\nPROCEED, RESET,\nKILLALL, ANNOTATE,\nSEND_ROBOT_*
TC2 <-right-> SERVER : **:5001**\nLarge Data Transfer\nDrill Site Poses\nTrajectories (JSON)
TC3 <-right-> SERVER : **:5002**\nContinuous Stream\nImages + Annotations\nSegmented Results

SERVER -down-> CTRL
SERVER -down-> TRAJ
SERVER -down-> SEG

note right of AVP
  **IP:** Dynamic (Vision Pro)
  **Connection:** Client
  **Language:** Swift
end note

note left of ROS
  **IP:** 192.168.0.193
  **Connection:** Server
  **Language:** Python
end note

@enduml
```

---

## Port Summary

| Port | Direction | Type | Purpose | Data Format |
|------|-----------|------|---------|-------------|
| **5000** | AVP → ROS | Command | One-time control commands | Text (command strings) |
| **5001** | ROS → AVP | Data | Large data transfer (drill poses) | JSON (trajectories) |
| **5002** | AVP ↔ ROS | Stream | Continuous image streaming | Binary (PNG) + JSON |

---

## Key Features

- **Port 5000 (Control)**: Workflow state transitions, emergency stop, annotation triggers
- **Port 5001 (Drill Poses)**: Receives computed drill trajectories from planning system
- **Port 5002 (Images)**: Bidirectional - sends images for segmentation, receives results