# Migration Guide: Single Node → Distributed Architecture

## Quick Summary

| Aspect | Old (app1_v2.py) | New (Multi-node) |
|--------|------------------|-----------------|
| Main loop | Single thread with locks | Three independent nodes |
| State machine | machine_loop() thread | StateManager event-driven |
| Inference | Blocking calls in machine loop | Async in separate node |
| Bottleneck | All in AppNode with threading | Distributed workload |
| Sensor data loss | Possible under load | Minimal (fast StateManager) |

## What Changed

### Old Architecture (app1_v2.py)
```
sensor_callback (high freq)
    ↓ (lock)
machine_loop (thread)
    ├─ State machine
    ├─ ML inference (BLOCKING)
    └─ Update plotter
    
Problems:
- Inference blocks sensor buffering
- Lock contention
- Plotter updates blocked during ML
```

### New Architecture (3 nodes)
```
Sensor → StateManager (fast) → Window
                                  ↓
                            Inference (can be slow)
                                  ↓
                                Velocity → App (plotting)
                                
Benefits:
- StateManager can run at full sensor rate
- Inference doesn't block sensor buffering
- App can update UI at its own rate
```

## Code Mapping

### sensor_callback → StateManager

**OLD:**
```python
def sensor_callback(self, msg):
    data = np.array(msg.data)
    with self.lock:
        self.buffer = np.roll(self.buffer, -1, axis=1)
        self.buffer[:, -1] = data
        self.sm.callback(self.buffer)
        
        if self.window_cnt < self.window_size:
            self.window[:, self.window_cnt] = data
            self.window_cnt += 1
        
        self.plotter.callback(data)
```

**NEW (StateManager):**
```python
def sensor_callback(self, msg):
    data = np.array(msg.data, dtype=np.float32)
    
    with self.lock:
        self.buffer = np.roll(self.buffer, -1, axis=1)
        self.buffer[:, -1] = data
        
        if self.window_cnt < self.window_size:
            self.window[:, self.window_cnt] = data
            self.window_cnt += 1
    
    # Calculate metrics and publish
    touch_metric = self.metric_touch()
    self.touch_metric_pub.publish(Float32(data=touch_metric))
    
    # Trigger state update
    self._update_state(touch_metric, self.metric_slide())
```

### machine_loop → StateManager + Inference

**OLD machine_loop:**
```python
def machine_loop(self):
    while rclpy.ok():
        with self.lock:
            data_window = self.window.copy()
            state = self.sm.state

        if state == 'idle':
            if self.sm.metric_touch() > 150:
                self.sm.state = 'touch'
        
        elif state == 'touch':
            # BLOCKING inference here!
            input_arr = self.sm.touch_buffer.reshape(1, 4, 1000)
            pose = self.touchmodel(torch.from_numpy(input_arr))
            self.plotter.pose = pose  # Update plotter
            
            with self.lock:
                self.window_cnt = 0
```

**NEW split:**

StateManager (non-blocking state transitions):
```python
def _update_state(self, touch_metric, slide_metric):
    # Fast state transitions only
    if self.state == 'idle' and touch_metric > 150:
        self.state = 'touch'  # Just change state
        self.state_pub.publish(String(data='touch'))
```

Inference node (handles blocking ML):
```python
def _inference_loop(self):
    while rclpy.ok():
        if state == 'touch':
            # Blocking operation happens here, doesn't affect sensor buffering
            vel = self._infer_touch(touch_buffer)
            self.velocity_pub.publish(Float32MultiArray(data=vel))
```

## Communication

### Old: Direct Python function calls
```python
self.sm.callback(self.buffer)  # Direct call
pose = self.touchmodel(input_tensor)  # Blocking
self.plotter.trajectory = np.vstack([self.plotter.trajectory, vel])
```

### New: ROS2 pub/sub messaging
```python
# StateManager publishes
self.touch_metric_pub.publish(Float32(data=metric))
self.state_pub.publish(String(data=self.state))
self.window_pub.publish(Float32MultiArray(data=window.flatten().tolist()))

# Inference node subscribes and processes
# App node subscribes to results
```

## Thread Safety

### Old (Multiple locks)
```python
self.lock = threading.Lock()
# In sensor_callback, machine_loop, and every access point
with self.lock:
    self.buffer = ...
    self.window = ...
```

### New (Per-node locks)
```python
# StateManager has its own lock - only protects its data
# Inference node has its own lock - only protects its data
# App node has its own lock - only protects trajectory
# NO cross-node locks needed - ROS2 message queue handles synchronization
```

## Testing/Debugging

### Monitor Topics

```bash
# Watch state transitions
ros2 topic echo /tribo/state

# Check window publishing (every 50ms at 1kHz sensor rate)
ros2 topic hz /tribo/window_buffer

# Monitor velocity output
ros2 topic echo /tribo/velocity

# List all active topics
ros2 topic list -t
```

### Run Individual Nodes

```bash
# Test StateManager independently
ros2 run tribo_plot state_manager

# In another terminal, check it's publishing
ros2 topic echo /tribo/state
```

### Add Logging

Each node has logging configured:
```python
self.get_logger().info(f'STATE: {self.state}')
self.get_logger().debug(f'Inference: vel={vel}')
self.get_logger().error(f'Error: {e}')
```

Change log level in launch file:
```python
Node(
    package='tribo_plot',
    executable='state_manager',
    output='screen',
    parameters=[{'log_level': 'DEBUG'}],
)
```

## Performance Tips

1. **Monitor window publish rate** - Should be 20 Hz (50ms between windows)
   ```bash
   ros2 topic hz /tribo/window_buffer
   ```

2. **Profile inference timing** - Add timing code:
   ```python
   import time
   start = time.time()
   vel = self.slidemodel(input_tensor)
   elapsed = time.time() - start
   self.get_logger().info(f'Inference: {elapsed*1000:.1f}ms')
   ```

3. **Check StateManager utilization** - Should be <10%
   ```bash
   # Monitor CPU usage
   top -p $(pgrep -f "state_manager")
   ```

4. **Optimize window size** - Balance between:
   - Smaller window (faster inference, lower latency)
   - Larger window (better model accuracy, higher latency)

## Rollback to Single Node

If you need to go back to single-node architecture, the original code is in git history:
```bash
git log --oneline  # Find old version
git show <hash>:app1_v2.py  # View old code
```

## Common Issues & Solutions

### Issue: Inference not running
**Cause**: Inference node not subscribed properly
**Solution**: Check topic names match exactly
```bash
ros2 topic list | grep tribo
```

### Issue: Plotter not updating
**Cause**: App node not receiving velocity messages
**Solution**: 
```bash
ros2 topic echo /tribo/velocity  # Check if messages published
```

### Issue: State machine stuck
**Cause**: StateManager not publishing state updates
**Solution**:
```bash
ros2 topic echo /tribo/state  # Check state messages
```

## Next Steps

1. Test with fake sensor: `ros2 launch tribo_plot tribo_app.launch.py`
2. Monitor topics: `ros2 topic list`
3. Add custom message types for type safety
4. Implement proper error handling for model loading
5. Add QoS profiles for reliability
