

# input

```c++
ErrorType planOnce(const Vehicle &raw_ego_vehicle, const std::vector<simulator::Lane> &raw_lanes, const std::vector<simulator::Vehicle> other_vehicles, const int aim_lane_id, double planning_horizon, VehicleControlSignal &control_signal)
```

+ const Vehicle &raw_ego_vehicle

  ```c++
  int id_{kInvalidAgentId};
      std::string subclass_;
      std::string type_;
      VehicleParam param_;
      State state_;
  ```

  ```
  ego vehicle state: 
  State:
   -- time_stamp: 0.000000.
   -- vec_position: (-1.340000, 12.000000).
   -- angle: 0.000000.
   -- curvature: 0.000000.
   -- velocity: 20.000000.
   -- acceleration: -0.000000.
   -- steer: 0.000000.
  semantic ego vehicle state: 
  
  Vehicle:
   -- ID:	0
   -- Subclass:	
  VehicleParam:
   -- width:	 1.900000.
   -- length:	 4.880000.
   -- wheel_base:	 2.850000.
   -- front_suspension:	 0.930000.
   -- rear_suspension:	 1.100000.
   -- d_cr:	 1.340000.
   -- max_steering_angle:	 45.000000.
   -- max_longitudinal_acc:	 2.000000.
   -- max_lateral_acc:	 2.000000.
  
  ```

+ const std::vector<simulator::Vehicle> other_vehicles

  ```
  Vehicle:
   -- ID:	1
   -- Subclass:	
  VehicleParam:
   -- width:	 2.000000.
   -- length:	 5.000000.
   -- wheel_base:	 2.970000.
   -- front_suspension:	 0.930000.
   -- rear_suspension:	 1.100000.
   -- d_cr:	 1.400000.
   -- max_steering_angle:	 45.000000.
   -- max_longitudinal_acc:	 2.000000.
   -- max_lateral_acc:	 2.000000.
  State:
   -- time_stamp: 0.000000.
   -- vec_position: (6.957863, 4.030085).
   -- angle: -0.004441.
   -- curvature: 0.000000.
   -- velocity: 13.791000.
   -- acceleration: 0.000000.
   -- steer: 0.000000.
  semantic vehicle state i: 2
  
  Vehicle:
   -- ID:	2
   -- Subclass:	
  VehicleParam:
   -- width:	 2.000000.
   -- length:	 5.000000.
   -- wheel_base:	 2.970000.
   -- front_suspension:	 0.930000.
   -- rear_suspension:	 1.100000.
   -- d_cr:	 1.400000.
   -- max_steering_angle:	 45.000000.
   -- max_longitudinal_acc:	 2.000000.
   -- max_lateral_acc:	 2.000000.
  State:
   -- time_stamp: 0.000000.
   -- vec_position: (39.193140, 0.000000).
   -- angle: 0.000000.
   -- curvature: 0.000000.
   -- velocity: 13.684865.
   -- acceleration: 0.000000.
   -- steer: 0.000000.
  semantic vehicle state i: 3
  
  Vehicle:
   -- ID:	3
   -- Subclass:	
  VehicleParam:
   -- width:	 2.000000.
   -- length:	 5.000000.
   -- wheel_base:	 2.970000.
   -- front_suspension:	 0.930000.
   -- rear_suspension:	 1.100000.
   -- d_cr:	 1.400000.
   -- max_steering_angle:	 45.000000.
   -- max_longitudinal_acc:	 2.000000.
   -- max_lateral_acc:	 2.000000.
  State:
   -- time_stamp: 0.000000.
   -- vec_position: (65.792409, 11.851343).
   -- angle: 0.021757.
   -- curvature: 0.000000.
   -- velocity: 14.006991.
   -- acceleration: 0.000000.
   -- steer: 0.000000.
  semantic vehicle state i: 4
  
  Vehicle:
   -- ID:	4
   -- Subclass:	
  VehicleParam:
   -- width:	 2.000000.
   -- length:	 5.000000.
   -- wheel_base:	 2.970000.
   -- front_suspension:	 0.930000.
   -- rear_suspension:	 1.100000.
   -- d_cr:	 1.400000.
   -- max_steering_angle:	 45.000000.
   -- max_longitudinal_acc:	 2.000000.
   -- max_lateral_acc:	 2.000000.
  State:
   -- time_stamp: 0.000000.
   -- vec_position: (115.159784, 4.000000).
   -- angle: 0.000000.
   -- curvature: 0.000000.
   -- velocity: 14.879204.
   -- acceleration: 0.000000.
   -- steer: 0.000000.
  semantic vehicle state i: 5
  
  ```

  