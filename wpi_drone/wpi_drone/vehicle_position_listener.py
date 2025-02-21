#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from terrawarden_interfaces.msg import DroneTelemetry, DroneWaypoint
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, BatteryStatus, SensorGps
from datetime import datetime
import math

# most coordinates are in NED (North, East, Down) frame, local coordinate offset inputs are in FLU (Forward, Left, Up) frame
class VehicleGlobalPositionListener(Node):
    def __init__(self):
        super().__init__('vehicle_global_position_listener')
        
        # Add debug parameters
        self.declare_parameter('debug_printout', True)
        self.declare_parameter('debug_printout_period_s', 2.0)
        
        # Add missing parameter declarations
        self.declare_parameter('drone_pose_topic', 'drone/waypoint')
        self.declare_parameter('drone_telemetry_topic', 'drone/telemetry')
        
        # Configure QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # maybe TRANSIENT_LOCAL
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Debug logging
        debug_printout = self.get_parameter('debug_printout').value
        if debug_printout:
            self.get_logger().info("Starting subscriber with QoS profile:")
            self.get_logger().info(f"Reliability: {qos_profile.reliability}")
            self.get_logger().info(f"Durability: {qos_profile.durability}")
            self.get_logger().info(f"History: {qos_profile.history}")
            self.get_logger().info(f"Depth: {qos_profile.depth}")

        # Internal state tracking
        self.isArmed = False
        self.isFlying = False
        self.isOffboard = False
        self.arm_timestamp = None
        self.flight_start_timestamp = None
        self.initialization_counter = 0     # after detecting vehicle is in onboard mode, wait for 5 seconds before taking autonomous control
        self.home_coord_offset = [0.0, 0.0, 0.0]
        self.last_traj_setpoint_msg_PX4 = None        
        self.status_var_temp = 0

        # PX4 publishers
        self.offboard_control_mode_publisher = self.create_publisher(   # heartbeat
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(         
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(     # offboard position sender
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # PX4 subscribers
        self.vehicle_status = None
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_odometry = None
        self.battery_status = None
        self.sensor_gps = None
        
        self.vehicle_status_subscriber = self.create_subscription( 
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)    
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.battery_status_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self.battery_status_callback, qos_profile)
        self.sensor_gps_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/sensor_gps', self.sensor_gps_callback, qos_profile)
        
        # ROS2 publishers
        self.publish_drone_telemetry_publisher = self.create_publisher(
            DroneTelemetry, self.get_parameter('drone_telemetry_topic'), 10)
        self.telemetry_timer = self.create_timer(0.1, self.publish_drone_telemetry)  # 10Hz
        
        # ROS2 subscibers
        self.waypoing_subscriber = self.create_subscription(
            DroneWaypoint,  self.get_parameter('drone_pose_topic'), self.waypoint_callback, qos_profile)
        

        
        self.heartbeat_timer = self.create_timer(0.1, self.timer_callback)
        
        if self.get_parameter('debug_printout').value:
            period = self.get_parameter('debug_printout_period_s').value
            self.timer_debug = self.create_timer(period, self.timer_debug_callback)

    # https://docs.px4.io/main/en/msg_docs/VehicleLocalPosition.html
    #     px4_msgs.msg.VehicleLocalPosition(
    #     timestamp=1738781937874852,
    #     timestamp_sample=1738781937874186,
    #     xy_valid=True,
    #     z_valid=True,
    #     v_xy_valid=True,
    #     v_z_valid=True,
    #     x=-0.21537010371685028,
    #     y=0.09170171618461609,
    #     z=-0.019485270604491234,
    #     delta_xy=array([5.0181836e-05, 2.7746145e-04], dtype=float32),
    #     xy_reset_counter=2,
    #     delta_z=0.0,
    #     z_reset_counter=1,
    #     vx=0.0025058595929294825,
    #     vy=0.005207868292927742,
    #     vz=-0.003228991525247693,
    #     z_deriv=-0.001445640460588038,
    #     delta_vxy=array([0.00742345, 0.00078792], dtype=float32),
    #     vxy_reset_counter=2,
    #     delta_vz=-0.00030795918428339064,
    #     vz_reset_counter=1,
    #     ax=0.020625105127692223,
    #     ay=0.04751252010464668,
    #     az=-0.09083916246891022,
    #     heading=1.1044014692306519,
    #     heading_var=0.005346738267689943,
    #     unaided_heading=0.009717458859086037,
    #     delta_heading=1.360266923904419,
    #     heading_reset_counter=1,
    #     heading_good_for_control=False,
    #     tilt_var=6.0623140598181635e-05,
    #     xy_global=False,
    #     z_global=False,
    #     ref_timestamp=0,
    #     ref_lat=nan,
    #     ref_lon=nan,
    #     ref_alt=nan,
    #     dist_bottom_valid=True,
    #     dist_bottom=0.1713714599609375,
    #     dist_bottom_var=0.000816226121969521,
    #     delta_dist_bottom=0.0,
    #     dist_bottom_reset_counter=1,
    #     dist_bottom_sensor_bitfield=1,
    #     eph=0.5449826717376709,
    #     epv=0.03457729145884514,
    #     evh=0.026098940521478653,
    #     evv=0.032592181116342545,
    #     dead_reckoning=False,
    #     vxy_max=0.68548583984375,
    #     vz_max=inf,
    #     hagl_min=0.07999999821186066,
    #     hagl_max=7.199999809265137
    # )
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position        
    
    # https://docs.px4.io/main/en/msg_docs/VehicleStatus.html
    # px4_msgs.msg.VehicleStatus(
    #     timestamp=1738782750834039,
    #     armed_time=0,
    #     takeoff_time=0,
    #     arming_state=1,
    #     latest_arming_reason=13,
    #     latest_disarming_reason=6,
    #     nav_state_timestamp=1448486364,
    #     nav_state_user_intention=2,
    #     nav_state=2,
    #     executor_in_charge=0,
    #     valid_nav_states_mask=2147411071,
    #     can_set_nav_states_mask=8307839,
    #     failure_detector_status=0,
    #     hil_state=0,
    #     vehicle_type=1,
    #     failsafe=False,
    #     failsafe_and_user_took_over=False,
    #     failsafe_defer_state=0,
    #     gcs_connection_lost=False,
    #     gcs_connection_lost_counter=1,
    #     high_latency_data_link_lost=False,
    #     is_vtol=False,
    #     is_vtol_tailsitter=False,
    #     in_transition_mode=False,
    #     in_transition_to_fw=False,
    #     system_type=2,
    #     system_id=1,
    #     component_id=1,
    #     safety_button_available=True,
    #     safety_off=True,
    #     power_input_valid=True,
    #     usb_connected=False,
    #     open_drone_id_system_present=False,
    #     open_drone_id_system_healthy=False,
    #     parachute_system_present=False,
    #     parachute_system_healthy=False,
    #     avoidance_system_required=False,
    #     avoidance_system_valid=False,
    #     rc_calibration_in_progress=False,
    #     calibration_enabled=False,
    #     pre_flight_checks_pass=True
    # )
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        # self.get_logger().info(f"Vehicle status: {vehicle_status.takeoff_time}")  # used for development prints
        self.vehicle_status = vehicle_status
        self.isArmed = vehicle_status.arming_state == 2 # armed
        self.isFlying = vehicle_status.takeoff_time > 0 # flying
        self.isOffboard = vehicle_status.nav_state == 14 # offboard
        
        # save the timestamp of the first time the vehicle is armed
        if self.isArmed:
            if self.arm_timestamp is None:
                self.arm_timestamp = vehicle_status.timestamp
        else:
            self.arm_timestamp = None
            self.initialization_counter = 0
            
        # save the timestamp and home coordinate offset
        if self.isFlying:
            if self.flight_start_timestamp is None:
                self.flight_start_timestamp = vehicle_status.timestamp
                self.home_coord_offset = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]
        else:
            self.flight_start_timestamp = None
            
    # https://docs.px4.io/main/en/msg_docs/VehicleOdometry.html
    # timestamp=1739808861325580,
    # timestamp_sample=1739808861324953,
    # pose_frame=1,
    # position=array([ 0.15550749, -0.02187049, -0.5224337 ], dtype=float32),
    # q=array([-0.18065909, 0.00464101, 0.00498464, 0.9835222 ], dtype=float32),
    # velocity_frame=1, velocity=array([ 4.5273470e-04, -3.4711199e-04, 7.1771887e-05], dtype=float32),
    # angular_velocity=array([-0.00037794, -0.00055995, -0.00013416], dtype=float32),
    # position_variance=array([0.0482139 , 0.04889397, 0.00182111], dtype=float32),
    # orientation_variance=array([1.6147900e-05, 1.6462798e-05, 6.9236341e-03], dtype=float32),
    # velocity_variance=array([0.00030635, 0.00030647, 0.00099156], dtype=float32),
    # reset_counter=6,
    # quality=0
    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = msg

    # https://docs.px4.io/main/en/msg_docs/BatteryStatus.html
    def battery_status_callback(self, msg):
        """Callback function for battery_status topic subscriber."""
        self.battery_status = msg

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Publish the heartbeat
        self.publish_offboard_control_heartbeat_signal()

        # Wait for vehicle status to be initialized
        if self.vehicle_status is None:
            return
                    
        # Wait for vehicle to be armed, flying, and in offboard mode
        if self.isArmed and self.isFlying and self.isOffboard:
            # count to 3 seconds equivalent
            if self.initialization_counter >= 30:
                if self.status_var_temp == 0:
                    self.get_logger().info("Vehicle is armed, flying, and in offboard mode")
                    self.status_var_temp = 1
                                                   
            else:
                self.initialization_counter += 1   
                
        
    def sensor_gps_callback(self, msg):
        """Callback function for sensor_gps topic subscriber."""
        self.sensor_gps = msg
        
    
    def timer_debug_callback(self) -> None:
        if self.vehicle_status is None:
            return        

        # self.get_logger().info(f"Vehicle status: {self.vehicle_local_position}")  # used for development prints
        
        # Build the status report as a single multi-line string
        report = []
        report.append("\n=== Vehicle Status Report ===")
        
        # Arming Status
        report.append("--- Arming Status ---")
        arming_state = "ARMED ✅" if self.vehicle_status.arming_state == 2 else "DISARMED ❌"
        report.append(f"State: {arming_state}")
        
        if self.vehicle_status:
            current_time = self.vehicle_status.timestamp
        else: 
            current_time = 0
        current_time_formatted = datetime.fromtimestamp(current_time / 1e6).strftime('%d/%m/%Y %H:%M:%S')
        report.append(f"[s] Current time: {current_time_formatted}")
            
        if self.arm_timestamp:
            armed_duration = int(current_time - self.arm_timestamp) / 1e6  # Convert microseconds to seconds    
        else:
            armed_duration = 0
        report.append(f"[s] Armed time: {armed_duration:.0f}")
        
        if self.flight_start_timestamp:            
            flight_duration = int(current_time - self.flight_start_timestamp) / 1e6  # Convert microseconds to seconds        
        else:
            flight_duration = 0
        report.append(f"[s] Flight time: {flight_duration:.0f}")
        
        # Arming/disarming reasons
        arm_disarm_reasons = {
            0: "Transition to Standby", 1: "Stick Gesture", 2: "RC Switch",
            3: "Command Internal", 4: "Command External", 5: "Mission Start",
            6: "Safety Button", 7: "Auto Disarm Land", 8: "Auto Disarm Preflight",
            9: "Kill Switch", 10: "Lockdown", 11: "Failure Detector",
            12: "Shutdown", 13: "Unit Test"
        }
        
        if self.vehicle_status.arming_state == 1:
            reason = arm_disarm_reasons.get(self.vehicle_status.latest_disarming_reason, "Unknown")
            report.append(f"Last disarm reason: {reason}\n")
        elif self.vehicle_status.arming_state == 2:
            reason = arm_disarm_reasons.get(self.vehicle_status.latest_arming_reason, "Unknown")
            report.append(f"Last arm reason: {reason}\n")
            
        # Navigation State
        report.append("--- Navigation Status ---")
        navigation_states = {
            0: "Manual - Direct manual control via RC",
            1: "Altitude Control - Altitude stabilization",
            2: "Position Control - Position control with stabilization",
            3: "Auto Mission - Autonomous mission execution",
            4: "Auto Loiter - Holding position automatically",
            5: "Auto Return to Launch - Returning to launch position",
            10: "Acro - Raw rate control mode",
            12: "Descend - Controlled descent mode",
            13: "Termination - Emergency stop",
            14: "Offboard - External control mode",
            15: "Stabilized - Basic attitude stabilization",
            17: "Auto Takeoff - Automatic takeoff sequence",
            18: "Auto Land - Automatic landing sequence",
            19: "Auto Follow Target - Following a designated target",
            20: "Auto Precision Land - Precision landing mode",
            21: "Orbit - Orbital flight around a point",
            22: "Auto VTOL Takeoff - Vertical takeoff for VTOL aircraft"
        }
        navigation_state = navigation_states.get(self.vehicle_status.nav_state, f"Unknown ({self.vehicle_status.nav_state})")
        report.append(f"Current mode: {navigation_state}\n")
        
        # System Health
        report.append("--- System Health ---")
        if self.vehicle_status.failure_detector_status != 0:
            failures = []
            failures.append("FAILURES DETECTED:")
            if self.vehicle_status.failure_detector_status & 1:
                failures.append("- Roll failure")
            if self.vehicle_status.failure_detector_status & 2:
                failures.append("- Pitch failure")
            if self.vehicle_status.failure_detector_status & 4:
                failures.append("- Altitude failure")
            if self.vehicle_status.failure_detector_status & 8:
                failures.append("- External failure")
            if self.vehicle_status.failure_detector_status & 16:
                failures.append("- ARM ESC failure")
            if self.vehicle_status.failure_detector_status & 32:
                failures.append("- Battery failure")
            if self.vehicle_status.failure_detector_status & 64:
                failures.append("- Imbalanced prop")
            if self.vehicle_status.failure_detector_status & 128:
                failures.append("- Motor failure")
            report.append("\n".join(failures))
        else:
            report.append("No failures detected")
            
        # Add Battery Status
        if self.battery_status:
            report.append("\n--- Battery Status ---")
            remaining = self.battery_status.remaining * 100
            report.append(f"Voltage: {self.battery_status.voltage_v:.2f}V")
            report.append(f"Current: {self.battery_status.current_a:.2f}A")
            report.append(f"Remaining: {remaining:.1f}%")
            report.append(f"Consumed: {self.battery_status.discharged_mah:.0f}mAh")
            if self.battery_status.warning > 0:
                warnings = []
                if self.battery_status.warning >= 1:
                    warnings.append("LOW BATTERY WARNING ⚠️")
                if self.battery_status.warning >= 2:
                    warnings.append("CRITICAL BATTERY WARNING ❌")
                report.append("Warnings: " + ", ".join(warnings))
        
        # Communication Status
        report.append("\n--- Communication Status ---")
        gcs_emoji = "📡✅" if not self.vehicle_status.gcs_connection_lost else "📡❌"
        report.append(f"GCS Connection: {'LOST' if self.vehicle_status.gcs_connection_lost else 'OK'} {gcs_emoji}")
        if self.vehicle_status.gcs_connection_lost:
            report.append(f"Connection lost count: {self.vehicle_status.gcs_connection_lost_counter}")
        
        # Safety Status
        report.append("\n--- Safety Status ---")
        report.append(f"Pre-flight checks: {'PASS ✅' if self.vehicle_status.pre_flight_checks_pass else 'FAIL ❌'}")
        
        # Position Status
        report.append("\n--- Position Status ---")
        dead_reckoning_emoji = "🛰️❌" if self.vehicle_local_position.dead_reckoning else "🛰️✅"
        report.append(f"Dead reckoning: {self.vehicle_local_position.dead_reckoning} {dead_reckoning_emoji}")
        report.append("\nValidity Checks:")
        report.append(f"{'Sensor':20} {'Valid?':<10} {'Status'}")
        report.append("-" * 40)
        report.append(f"{'Terrain altitude':20} {str(self.vehicle_local_position.dist_bottom_valid):<10} {'✅' if self.vehicle_local_position.dist_bottom_valid else '❌'}")
        report.append(f"{'Position XY':20} {str(self.vehicle_local_position.xy_valid):<10} {'✅' if self.vehicle_local_position.xy_valid else '❌'}")
        report.append(f"{'Position Z':20} {str(self.vehicle_local_position.z_valid):<10} {'✅' if self.vehicle_local_position.z_valid else '❌'}")
        
        report.append("\nPosition Data:")
        report.append(f"{'Parameter':25} {'Value':>10} {'Uncertainty StdDev σ':>10}")
        report.append("-" * 45)
        report.append(f"{'[m] Terrain altitude':25} {self.vehicle_local_position.dist_bottom:>10.3f} {self.vehicle_local_position.dist_bottom_var:>10.3f}")
        report.append(f"{'[m] Local NED North':25} {self.vehicle_local_position.x:>10.3f} {self.vehicle_local_position.eph:>10.3f}")
        report.append(f"{'[m] Local NED East':25} {self.vehicle_local_position.y:>10.3f} {self.vehicle_local_position.eph:>10.3f}")
        report.append(f"{'[m] Local NED Z':25} {self.vehicle_local_position.z:>10.3f} {self.vehicle_local_position.epv:>10.3f}")
        
        report.append("\nGlobal Position:")
        report.append(f"{'Parameter':20} {'Valid?':<8} {'Value'}")
        report.append("-" * 40)
        report.append(f"{'Global XY':20} {str(self.vehicle_local_position.xy_global):<8} {self.vehicle_local_position.ref_lat:>.3f}, {self.vehicle_local_position.ref_lon:.3f}")
        report.append(f"{'Global Z':20} {str(self.vehicle_local_position.z_global):<8} {self.vehicle_local_position.ref_alt:.3f}")
        
        report.append("\nVelocity Data:")
        report.append(f"{'Parameter':25} {'Value':>10} {'Uncertainty Std Dev σ':>10}")
        report.append("-" * 45)
        report.append(f"{'[m/s] Local NED VNorth':25} {self.vehicle_local_position.vx:>10.3f} {self.vehicle_local_position.evh:>10.3f}")
        report.append(f"{'[m/s] Local NED VEast':25} {self.vehicle_local_position.vy:>10.3f} {self.vehicle_local_position.evh:>10.3f}")
        report.append(f"{'[m/s] Local NED VZ':25} {self.vehicle_local_position.vz:>10.3f} {self.vehicle_local_position.evv:>10.3f}")
        
        readable_heading = self.px4_yaw_to_heading(self.vehicle_local_position.heading)
        report.append(f"\n{'[deg] Global Heading':25} {readable_heading:>10.3f} {self.vehicle_local_position.heading_var * 57.2958:>10.3f}")
        
        self.get_logger().info("\n".join(report))

    
    # -----


    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
        
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        
    def publish_trajectory_setpoint(self, position: list[float, float, float], heading_deg:float = None, max_ang_vel_deg_s:float = None, max_lin_vel_m_s:float = None, max_z_vel_m_s:float = None, max_lin_accel_m_s2:float = None) -> None:
        """Publish a trajectory setpoint in drone NED with home offset applied
        Takes in a position list [x, y, z] in meters and a yaw in degrees as bearing
        Optional parameters for max angular velocity, linear velocity, z velocity, and linear acceleration
        If any velocity parameter is <= 0, drone will hold current position
        """
        msg = TrajectorySetpoint()

        if (max_lin_vel_m_s is not None and max_lin_vel_m_s <= 0) or \
           (max_z_vel_m_s is not None and max_z_vel_m_s <= 0) or \
           (max_ang_vel_deg_s is not None and max_ang_vel_deg_s <= 0):
            
            position= self.get_current_ned_pos()
            self.get_logger().warn("Invalid velocity parameters detected - holding position")

        msg.position = self.convert_to_PX4_ned_coordinates(position)
            
        if heading_deg is not None:
            msg.yaw = heading_deg * (math.pi / 180.0)  # Convert degrees to radians
        else:
            msg.yaw = self.vehicle_local_position.heading
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        if max_ang_vel_deg_s:
            msg.yawspeed = max_ang_vel_deg_s * (math.pi / 180.0)
            
        if max_lin_vel_m_s and max_z_vel_m_s:
            msg.velocity = [max_lin_vel_m_s, max_lin_vel_m_s, max_z_vel_m_s]
            
        if max_lin_accel_m_s2:
            msg.acceleration = [max_lin_accel_m_s2, max_lin_accel_m_s2, max_lin_accel_m_s2]      
        
        self.last_traj_setpoint_msg_PX4 = msg
        self.get_logger().info(f"Trajectory setpoint: {msg.position}")
        self.trajectory_setpoint_publisher.publish(msg)
        
    def waypoint_callback(self, msg):
        """Callback function for waypoint topic subscriber."""
        self.get_logger().info(f"Waypoint received: {msg}")            
        
        # publish the trajectory setpoint
        # --JJ I am not proud of this line of code, but it works
        self.publish_trajectory_setpoint([msg.ned_pos.x, msg.ned_pos.y, msg.ned_pos.z], msg.heading_degrees, msg.max_ang_vel_deg_s, msg.max_lin_vel_m_s, msg.max_z_vel_m_s, msg.max_lin_accel_m_s2)
        
    def publish_drone_telemetry(self) -> None:
        """Publish the drone telemetry."""
        msg = DroneTelemetry()
            
        if self.vehicle_local_position:
            # Position
            pos = PoseStamped()
            pos.pose.position.x = self.vehicle_local_position.x
            pos.pose.position.y = self.vehicle_local_position.y
            pos.pose.position.z = self.vehicle_local_position.z
            if self.vehicle_odometry:
                pos.pose.orientation.x = self.vehicle_odometry.q[0]
                pos.pose.orientation.y = self.vehicle_odometry.q[1]
                pos.pose.orientation.z = self.vehicle_odometry.q[2]
                pos.pose.orientation.w = self.vehicle_odometry.q[3]
            pos.header.stamp = self.get_clock().now().to_msg()
            msg.pos = pos
            
            # Velocity
            vel = PoseStamped()
            vel.pose.position.x = self.vehicle_local_position.vx
            vel.pose.position.y = self.vehicle_local_position.vy
            vel.pose.position.z = self.vehicle_local_position.vz
            vel.header.stamp = self.get_clock().now().to_msg()
            msg.vel = vel
            
        # GPS
        if self.sensor_gps:
            msg.gps = convert_px4_gps_to_navsat(self.sensor_gps)
        
        # Other
        msg.is_flying = self.isFlying
        msg.is_offboard = self.isOffboard
        if self.vehicle_local_position:
            msg.altitude_above_ground = self.vehicle_local_position.dist_bottom
            msg.heading_degrees = self.px4_yaw_to_heading(self.vehicle_local_position.heading)
        if self.battery_status:
            msg.battery_percentage = self.battery_status.remaining * 100
                           
        # battery low
        if self.battery_status:
            if self.battery_status.warning > 0:
                msg.error = "Battery low"
        else:
            msg.error = "TODO: make the error messages 📎"
    
        
        # Convert PX4 sensor_gps to ROS2 NavSatFix
        def convert_px4_gps_to_navsat(px4_gps):
            navsat = NavSatFix()
                
            navsat.latitude = px4_gps.latitude_deg * 1e-7  # PX4 uses degrees * 1e7
            navsat.longitude = px4_gps.longitude_deg * 1e-7  # PX4 uses degrees * 1e7
            navsat.altitude = px4_gps.altitude_ellipsoid_m
            
            # Covariance matrix (9 elements for 3x3 matrix)
            # PX4 provides eph (horizontal position error) and epv (vertical position error)
            # Convert from m^2 to standard deviation squared
            navsat.position_covariance[0] = px4_gps.eph * px4_gps.eph  # xx
            navsat.position_covariance[4] = px4_gps.eph * px4_gps.eph  # yy
            navsat.position_covariance[8] = px4_gps.epv * px4_gps.epv  # zz
            
            # Cross-terms are set to 0 as PX4 doesn't provide correlation data
            navsat.position_covariance[1] = 0.0  # xy
            navsat.position_covariance[2] = 0.0  # xz
            navsat.position_covariance[3] = 0.0  # yx
            navsat.position_covariance[5] = 0.0  # yz
            navsat.position_covariance[6] = 0.0  # zx
            navsat.position_covariance[7] = 0.0  # zy
            
            # set to uint8 COVARIANCE_TYPE_UNKNOWN = 0 if no GPS, and uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2 if lock
            if self.sensor_gps.fix_type >= 3:
                navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            else:
                navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            return navsat
        pass

        
    # -----
    
    def heading_to_px4_yaw(self, heading: float) -> float:
        """Convert a heading in degrees [0, 360) to a PX4 yaw in radians (-pi, pi]
        Heading is in degrees, yaw is in radians"""
        heading = heading % 360.0
        if heading > 180.0:
            heading = heading - 360.0
        return heading * (math.pi / 180.0) # heading is backwards 
    
    def px4_yaw_to_heading(self, yaw: float) -> float:
        """Convert a PX4 yaw in radians (-pi, pi] to a heading in degrees [0, 360)
        Yaw is in radians, heading is in degrees"""
        return (yaw * (180.0 / math.pi)) % 360.0
    
    # take in a local offset in meters and a yaw in degrees, convert to the NED frame, and return the NED coordinates
    def ned_point_from_flu_offset(self, curr_ned_pos: list[float, float, float], offset_flu: list[float, float, float]) -> list[float, float, float]:
        """Convert a local FLU offset in meters to NED coordinates
        Returns the offset in NED, not the global NED"""
        # convert yaw to radians
        yaw_current = self.vehicle_local_position.heading    
        
        # convert the offset to rotated FLU
        rotated_flu_x = offset_flu[0] * math.cos(yaw_current) + offset_flu[1] * math.sin(yaw_current)
        rotated_flu_y = offset_flu[0] * math.sin(yaw_current) - offset_flu[1] * math.cos(yaw_current)
        rotated_flu_z = offset_flu[2]
        
        # convert to NED coordinates
        offset_ned = [rotated_flu_x, rotated_flu_y, -rotated_flu_z]  
        return [curr_ned_pos[0] + offset_ned[0], curr_ned_pos[1] + offset_ned[1], curr_ned_pos[2] + offset_ned[2]]
    
    def check_terrain_safe(self, threshold=0.5) -> bool:
        """Check if the terrain is clear
        Threshold is the minimum distance to the terrain in meters
        Returns true if the terrain is safe, false otherwise"""
        if self.vehicle_local_position.dist_bottom > threshold and self.vehicle_local_position.dist_bottom_valid:
            return True
        else:
            return False
            
    def get_current_ned_pos(self) -> list[float, float, float]:
        """Get the current position in NED coordinates"""
        # autonatically offsets from the PX4 internal tracking system
        return [self.vehicle_local_position.x - self.home_coord_offset[0], self.vehicle_local_position.y - self.home_coord_offset[1], self.vehicle_local_position.z - self.home_coord_offset[2]]
    
    def get_traj_setpoint(self) -> list[float, float, float]:
        """Get the trajectory setpoint in NED coordinates
        Returns the trajectory setpoint in NED coordinates"""
        if self.last_traj_setpoint_msg_PX4 is None:
            return None
        return self.convert_from_PX4_ned_coordinates(self.last_traj_setpoint_msg_PX4.position)
    
    def convert_to_PX4_ned_coordinates(self, target: list[float, float, float]) -> list[float, float, float]:
        """Get the position in NED coordinates
        Returns the position in PX4 NED coordinates"""
        return [target[0] + self.home_coord_offset[0], target[1] + self.home_coord_offset[1], target[2] + self.home_coord_offset[2]]
    
    def convert_from_PX4_ned_coordinates(self, target: list[float, float, float]) -> list[float, float, float]:
        """Get the position in PX4 NED coordinates
        Returns the position in NED coordinates"""
        return [target[0] - self.home_coord_offset[0], target[1] - self.home_coord_offset[1], target[2] - self.home_coord_offset[2]]
    
    def is_at_position(self, target: list[float, float, float], threshold=0.5) -> bool:
        """Check if the vehicle is at a given position, threshold is in meters
        Returns true if the vehicle is at the position, false otherwise"""
        curr_pos = self.get_current_ned_pos()
        if math.sqrt((curr_pos[0] - target[0])**2 + (curr_pos[1] - target[1])**2 + (curr_pos[2] - target[2])**2) < threshold:
            return True
        else:
            return False
        
    def is_at_traj_setpoint(self, threshold=0.5) -> bool:
        """Check if the vehicle is at the trajectory setpoint
        Threshold is in meters
        Returns true if the vehicle is at the trajectory setpoint, false otherwise"""
        if self.last_traj_setpoint_msg_PX4 is None:
            return False
        return self.is_at_position(self.get_traj_setpoint(), threshold)
    # -----
                
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        
    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
        
    # -----
        

def main(args=None) -> None:
    print("Starting vehicle_global_position listener node...")
    rclpy.init(args=args)
    vehicle_global_position_listener = VehicleGlobalPositionListener()
    rclpy.spin(vehicle_global_position_listener)
    
    vehicle_global_position_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)