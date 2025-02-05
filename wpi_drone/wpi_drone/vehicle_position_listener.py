#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class VehicleGlobalPositionListener(Node):
    def __init__(self):
        super().__init__('vehicle_global_position_listener')
        
        # Configure QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # maybe TRANSIENT_LOCAL
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Add debug logging
        self.get_logger().info("Starting subscriber with QoS profile:")
        self.get_logger().info(f"Reliability: {qos_profile.reliability}")
        self.get_logger().info(f"Durability: {qos_profile.durability}")
        self.get_logger().info(f"History: {qos_profile.history}")
        self.get_logger().info(f"Depth: {qos_profile.depth}")
        debug_printout = True

        # Add initialization state tracking
        self.vehicle_status = None
        self.initialization_counter = 0
        self.arm_state = False
        self.vehicle_local_position = VehicleLocalPosition()

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscription
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        if debug_printout:
            self.timer_debug = self.create_timer(1.0, self.timer_debug_callback)

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
        self.vehicle_status = vehicle_status
        
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Publish the heartbeat
        self.publish_offboard_control_heartbeat_signal()

        # Wait for vehicle status to be initialized
        if self.vehicle_status is None:
            return
        pass
    
    def timer_debug_callback(self) -> None:
        if self.vehicle_status is None:
            return        
        print("\n\n\n=== Vehicle Status Report ===")
        
        # Arming Status
        print("\n--- Arming Status ---")
        arming_state = "ARMED" if self.vehicle_status.arming_state == 2 else "DISARMED"
        print(f"State: {arming_state}")
        
        # Print arming/disarming reason if relevant
        arm_disarm_reasons = {
            0: "Transition to Standby", 1: "Stick Gesture", 2: "RC Switch",
            3: "Command Internal", 4: "Command External", 5: "Mission Start",
            6: "Safety Button", 7: "Auto Disarm Land", 8: "Auto Disarm Preflight",
            9: "Kill Switch", 10: "Lockdown", 11: "Failure Detector",
            12: "Shutdown", 13: "Unit Test"
        }
        if self.vehicle_status.arming_state == 1:  # Disarmed
            reason = arm_disarm_reasons.get(self.vehicle_status.latest_disarming_reason, "Unknown")
            print(f"Last disarm reason: {reason}")
        elif self.vehicle_status.arming_state == 2:  # Armed
            reason = arm_disarm_reasons.get(self.vehicle_status.latest_arming_reason, "Unknown")
            print(f"Last arm reason: {reason}")
            armed_time = self.vehicle_status.armed_time / 1e6  # Convert to seconds from us
            print(f"Armed time: {armed_time:.1f} seconds")
            
        # Navigation State
        print("\n--- Navigation Status ---")
        navigation_states = {
            0: "Manual - Direct manual control via RC",
            1: "Altitude Control - Manual control with altitude stabilization",
            2: "Position Control - Manual position control with stabilization",
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
        print(f"Current mode: {navigation_state}")
        
        # System Health
        print("\n--- System Health ---")
        if self.vehicle_status.failure_detector_status != 0:
            print("FAILURES DETECTED:")
            if self.vehicle_status.failure_detector_status & 1:
                print("- Roll failure")
            if self.vehicle_status.failure_detector_status & 2:
                print("- Pitch failure")
            if self.vehicle_status.failure_detector_status & 4:
                print("- Altitude failure")
            if self.vehicle_status.failure_detector_status & 8:
                print("- External failure")
            if self.vehicle_status.failure_detector_status & 16:
                print("- ARM ESC failure")
            if self.vehicle_status.failure_detector_status & 32:
                print("- Battery failure")
            if self.vehicle_status.failure_detector_status & 64:
                print("- Imbalanced prop")
            if self.vehicle_status.failure_detector_status & 128:
                print("- Motor failure")
        else:
            print("No failures detected")

        # Communication Status
        print("\n--- Communication Status ---")
        print(f"GCS Connection: {'LOST' if self.vehicle_status.gcs_connection_lost else 'OK'}")
        if self.vehicle_status.gcs_connection_lost:
            print(f"Connection lost count: {self.vehicle_status.gcs_connection_lost_counter}")
        
        # Safety Status
        print("\n--- Safety Status ---")
        print(f"Safety switch: {'Present' if self.vehicle_status.safety_button_available else 'Not Present'}, " +
              f"{'SAFE' if not self.vehicle_status.safety_off else 'NOT SAFE'}")
        print(f"Pre-flight checks: {'PASS' if self.vehicle_status.pre_flight_checks_pass else 'FAIL'}")
        
        # Position Status
        print("\n--- Position Status ---")
        print(f"Dead reckoning?: {self.vehicle_local_position.dead_reckoning}")
        
        print(f"Terrain alt valid? {self.vehicle_local_position.dist_bottom_valid}, pos XY valid? {self.vehicle_local_position.xy_valid}, pos Z valid? {self.vehicle_local_position.z_valid}")
        print(f"Terrain altitude: {self.vehicle_local_position.dist_bottom}")
        print(f"Terrain altitude std. dev: {self.vehicle_local_position.dist_bottom_var}")
        
        print(f"Local NED pos x: {self.vehicle_local_position.x}, y: {self.vehicle_local_position.y}, z: {self.vehicle_local_position.z}")
        print(f"Pos std. dev XY: {self.vehicle_local_position.eph} Z: {self.vehicle_local_position.evh}")
        
        print("--- Global ---")
        print(f"Global XY valid? {self.vehicle_local_position.xy_global}, Z valid? {self.vehicle_local_position.z_global}")
        print(f"Global NED pos x: {self.vehicle_local_position.ref_lat}, y: {self.vehicle_local_position.ref_lon}, z: {self.vehicle_local_position.ref_alt}")
        
        print("--- Velocity ---")
        print(f"Local NED vx: {self.vehicle_local_position.vx}, vy: {self.vehicle_local_position.vy}, vz: {self.vehicle_local_position.vz}")
        print(f"Velocity std. dev XY: {self.vehicle_local_position.evh} Z: {self.vehicle_local_position.evv}")
    
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

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0)
        self.get_logger().info("Switching to offboard mode")


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