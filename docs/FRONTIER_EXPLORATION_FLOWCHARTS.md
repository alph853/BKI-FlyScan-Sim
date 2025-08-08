# Frontier Explorer Operational Flow Diagram

## System Overview

```mermaid
graph TB
    A[System Start] --> B[Configure Parameters]
    B --> C[Wait for Map & TF]
    C --> D{Map Available?}
    D -->|No| C
    D -->|Yes| E[Update Robot Position]
    
    E --> F[Frontier Detection Stage]
    F --> G[DFS Flood Fill within ROI]
    G --> H[DBSCAN Clustering]
    H --> I{Frontiers Found?}
    
    I -->|No| J[Adjust ROI Radius ++]
    J --> K{Exploration Complete?}
    K -->|Yes| L[END]
    K -->|No| E
    
    I -->|Yes| M[Calculate Utilities]
    M --> N[Sort by Utility Score]
    N --> O[Check Similarity Detection]
    O --> P{Similar to Recent?}
    P -->|Yes| Q[Skip Frontier]
    Q --> R{More Frontiers?}
    R -->|Yes| O
    R -->|No| J
    
    P -->|No| S[Publish Goal]
    S --> T[Adjust ROI Radius --]
    T --> U[Wait for Timer]
    U --> E
```

## Detailed Algorithm Flow

```mermaid
graph LR
    subgraph "Stage 1: Initialization"
        A1[Node Activation] --> A2[Set Autonomous Mode]
        A2 --> A3[Start Timer]
    end
    
    subgraph "Stage 2: Data Validation"
        B1[Check Map Available] --> B2[Update Robot Pose via TF]
        B2 --> B3{Valid Data?}
        B3 -->|No| B1
    end
    
    subgraph "Stage 3: Frontier Detection"
        C1[Initialize Visited Grid] --> C2[Scan ROI Cells]
        C2 --> C3[DFS Flood Fill]
        C3 --> C4[Collect Frontier Points]
    end
    
    subgraph "Stage 4: DBSCAN Clustering"
        D1[Calculate Distances] --> D2[Find Core Points]
        D2 --> D3[Expand Clusters]
        D3 --> D4[Filter by Min Size]
    end
    
    subgraph "Stage 5: Utility Optimization"
        E1[Distance Score] --> E4[Combined Utility]
        E2[Size Score] --> E4
        E3[Information Gain] --> E4
        E4 --> E5[Sort Frontiers]
    end
    
    subgraph "Stage 6: Goal Selection"
        F1[Check Utility Threshold] --> F2[Similarity Detection]
        F2 --> F3[Select Best Frontier]
        F3 --> F4[Publish Goal]
    end
    
    A3 --> B1
    B3 -->|Yes| C1
    C4 --> D1
    D4 --> E1
    E5 --> F1
    F4 --> B1
```

## Mathematical Operations Flow

```mermaid
graph TB
    subgraph "Frontier Detection Math"
        M1["F(x,y) = {free cell ∧ has unknown neighbor}"]
        M2["ROI(x,y) = ||P_robot - P(x,y)|| ≤ r_roi"]
        M1 --> M2
    end
    
    subgraph "DBSCAN Math"
        M3["d(p₁,p₂) = ||p₁ - p₂||₂"]
        M4["|N_ε(p)| ≥ min_points"]
        M3 --> M4
    end
    
    subgraph "Utility Function Math"
        M5["S_dist = 1/(1 + d/10)"]
        M6["S_size = min(|F|/20, 1)"]
        M7["S_info = N_unknown/N_total"]
        M8["U = w_d·S_dist + w_s·S_size + w_I·S_info"]
        M5 --> M8
        M6 --> M8
        M7 --> M8
    end
    
    subgraph "Adaptive ROI Math"
        M9["r_roi(t+1) = f(N_frontiers, r_roi(t))"]
        M10["Similarity = Σ[d ≤ θ]/|History| > 0.8"]
    end
    
    M2 --> M3
    M4 --> M5
    M8 --> M9
    M9 --> M10
```

## State Machine Transitions

```mermaid
stateDiagram-v2
    [*] --> Unconfigured
    Unconfigured --> Inactive : configure()
    Inactive --> Active : activate()
    Active --> Inactive : deactivate()
    Inactive --> Finalized : cleanup()
    Active --> ErrorProcessing : on_error()
    ErrorProcessing --> Inactive : on_recover()
    Finalized --> [*]
    
    state Active {
        [*] --> WaitingForData
        WaitingForData --> ProcessingFrontiers : map & tf ready
        ProcessingFrontiers --> PublishingGoal : frontiers found
        ProcessingFrontiers --> ExplorationComplete : no valid frontiers
        PublishingGoal --> WaitingForData : goal published
        ExplorationComplete --> [*]
    }
```

## Data Flow Architecture

```mermaid
graph LR
    subgraph "Inputs"
        I1[Map Topic<br>/map] 
        I2[TF Transform<br>map→base_link]
        I3[Parameters<br>YAML Config]
    end
    
    subgraph "Processing Core"
        P1[Frontier Explorer Node]
        P2[DFS Algorithm]
        P3[DBSCAN Clustering]
        P4[Utility Calculator]
        P5[Goal Selector]
    end
    
    subgraph "Outputs"
        O1[Goal Topic<br>/exploration_goal]
        O2[Service Calls<br>/set_control_mode]
        O3[Log Messages<br>INFO/DEBUG/WARN]
    end
    
    I1 --> P1
    I2 --> P1
    I3 --> P1
    P1 --> P2
    P2 --> P3
    P3 --> P4
    P4 --> P5
    P5 --> O1
    P1 --> O2
    P1 --> O3
```

## Performance Profiling Flow

```mermaid
graph TB
    subgraph "Timing Analysis"
        T1[Total Cycle Time: 50-200ms] --> T2[Frontier Detection: 10-50ms]
        T1 --> T3[DBSCAN: 5-20ms]
        T1 --> T4[Utility Calc: 1-5ms/frontier]
        T1 --> T5[Overhead: ~20ms]
    end
    
    subgraph "Memory Analysis"
        M1[Grid Processing: O(W×H)] --> M2[Visited Array]
        M1 --> M3[Frontier Storage: O(F×P)]
        M1 --> M4[History Buffer: O(H)]
    end
    
    subgraph "Complexity Analysis"
        C1[DFS: O(n)] --> C2[DBSCAN: O(n log n)]
        C2 --> C3[Utility: O(k×r²)]
        C3 --> C4[Total: O(n log n)]
    end
```
