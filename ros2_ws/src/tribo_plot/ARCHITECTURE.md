# Distributed Tribo Slide Application Architecture

## Overview
Your application has been refactored into a **distributed ROS2 architecture** with three specialized nodes communicating via publish/subscribe pattern. This solves the threading/bottleneck issues by:

1. **Separating concerns**: Each node has a single responsibility
2. **Efficient scheduling**: Each node runs at its optimal frequency
3. **Non-blocking operations**: One node's processing doesn't block another

## Node Architecture

### 1. **StateManager Node** (`state_manager_node.py`)
- **Role**: Sensor data acquisition, buffering, state machine
- **Frequency**: Runs at sensor frequency (833 Hz = 1.2ms period)
- **Subscribes to**:
  - `/sensorT_fake` (Float32MultiArray) - Raw sensor data [4 values]
- **Publishes**:
  - `/tribo/state` (String) - Current state: idle, touch, stay, slide, detach
  - `/tribo/touch_metric` (Float32) - Touch detection metric
  - `/tribo/slide_metric` (Float32) - Slide detection metric  
  - `/tribo/window_buffer` (Float32MultiArray) - Window data when full (4×50 → 200 values)

**Key Responsibilities**:
- Maintain rolling buffer (4, 1000)
- Fill non-overlapping window (4, 50)
- Calculate metrics for state transitions
- Publish window only when full (reduces data volume)
- Run state machine: idle → touch → stay/slide → detach → idle

### 2. **Inference Node** (`inference_node.py`)
- **Role**: ML model inference (computationally expensive)
- **Frequency**: Event-driven (runs only when window is published)
- **Subscribes to**:
  - `/tribo/window_buffer` (Float32MultiArray) - Window data from StateManager
  - `/tribo/state` (String) - Current state
  - `/tribo/touch_metric` (Float32) - For touch state handling (optional)
- **Publishes**:
  - `/tribo/velocity` (Float32MultiArray) - Model output [vx, vy]

**Key Responsibilities**:
- Load TouchNetwork and SlideNetwork models
- Run inference only when data is available
- Non-blocking tensor operations with torch.no_grad()
- Output results immediately after inference

### 3. **App Node** (`app_node.py`)
- **Role**: Visualization and trajectory plotting
- **Frequency**: UI-driven (updates when velocity received)
- **Subscribes to**:
  - `/tribo/velocity` (Float32MultiArray) - Velocity output [vx, vy]
  - `/tribo/state` (String) - Current state
- **Publishes**: (None)

**Key Responsibilities**:
- Maintain PyQt5 plotter window
- Stack velocity vectors to trajectory
- Reset trajectory on detach/idle
- Display real-time visualization

## Data Flow

```
Sensor Data (1.2ms)
    ↓
[StateManager] → /tribo/state
    ↓
    ├─→ /tribo/touch_metric → (used by StateManager for state transitions)
    ├─→ /tribo/slide_metric
    └─→ /tribo/window_buffer (when full, 50ms interval at 1kHz)
         ↓
      [Inference] → /tribo/velocity → [App] → Plotter
                                         ↓
                                      /tribo/state → Resets on idle
```

## Performance Benefits

### Before (Single-threaded with locks):
- All operations run on one thread with locks
- ML inference blocks sensor buffering
- Plotting blocks state machine
- Lock contention during high-frequency sensor callbacks

### After (Distributed nodes):
- **StateManager**: Runs at sensor frequency (833 Hz) - very fast, minimal processing
- **Inference**: Runs event-driven when window is full (20 Hz = 50ms) - can take 100ms without blocking
- **App**: Runs UI-driven - smooth visualization
- **No cross-node locks**: Each node manages its own data

## Timing Characteristics

| Operation | Frequency | Period | Notes |
|-----------|-----------|--------|-------|
| Sensor callback | 833 Hz | 1.2 ms | StateManager |
| Window publish | 20 Hz | 50 ms | When 50 samples collected |
| ML inference | ~10-20 Hz | 50-100 ms | Async when window received |
| UI updates | 60 Hz | 16.7 ms | PyQt5 event loop |

## Running the Application

### Launch all nodes together:
```bash
ros2 launch tribo_plot tribo_app.launch.py
```

### Or run each node separately:
```bash
# Terminal 1: State Manager
ros2 run tribo_plot state_manager

# Terminal 2: Inference
ros2 run tribo_plot inference

# Terminal 3: App (with visualization)
ros2 run tribo_plot app_node

# Terminal 4: Sensor (if using fake sensor)
ros2 run tribo_plot sensorT_fake
```

## Topic Monitoring

View published topics in real-time:
```bash
# Monitor state changes
ros2 topic echo /tribo/state

# Monitor velocity output
ros2 topic echo /tribo/velocity

# Check topic bandwidth
ros2 topic bw /tribo/window_buffer
```

## Customization Points

### Adjust StateManager Frequencies
Edit in `state_manager_node.py`:
```python
self.buffer_size = 1000     # Total history
self.window_size = 50       # Window for inference (increase for slower models)
```

### Adjust Thresholds
Edit in `state_manager_node.py`:
```python
if touch_metric > 150:      # Touch detection threshold
if touch_metric < -150:     # Detach detection threshold
if slide_metric < 20:       # Stay threshold
```

### Load Pre-trained Weights
Edit in `inference_node.py`:
```python
self.touchmodel = TouchNetwork()
# self.touchmodel.load_state_dict(torch.load('path/to/touch_weights.pt'))

self.slidemodel = SlideNetwork()
# self.slidemodel.load_state_dict(torch.load('path/to/slide_weights.pt'))
```

## Debugging Tips

1. **Check node status**:
   ```bash
   ros2 node list
   ```

2. **Monitor specific topic**:
   ```bash
   ros2 topic echo /tribo/state --once
   ```

3. **Check inference timing** (add logging):
   ```python
   start = time.time()
   vel = self.slidemodel(input_tensor)
   elapsed = time.time() - start
   self.get_logger().info(f'Inference took {elapsed*1000:.2f}ms')
   ```

4. **Use rqt_graph to visualize nodes**:
   ```bash
   rqt_graph
   ```

## Future Enhancements

1. **Custom Messages**: Replace Float32MultiArray with custom message types for type safety
2. **Quality of Service**: Configure QoS profiles for reliability vs latency trade-offs
3. **Action Servers**: Use ROS2 Actions for long-running touch inference
4. **Service Calls**: Add services for model loading/switching
5. **Parameter Server**: Use ROS2 parameters for dynamic threshold adjustment
