#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Lock
from std_msgs.msg import Bool


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, '/system_state', 10)
        self.arduino_cmd_pub = self.create_publisher(String, '/arduino_command', 10)
        
        self.controller_update = self.create_publisher(String, '/vision_node_update_signal', 10)
        self.controller_update.publish(String(data="idle"))  # Reset "low"

        # --- Subscribers ---
        self.create_subscription(String, '/kernel_state', self.kernel_state_callback, 10)

        # --- Internal state ---
        self.current_state = 'initializing'
        self.kernel_state = 'initializing'

        # --- Motor/device concurrency control ---
        self.device_states = {
            "singulator": {"lock": Lock(), "busy": False, "timer": None},
            "arm_servo": {"lock": Lock(), "busy": False, "timer": None},
            "dump_servo": {"lock": Lock(), "busy": False, "timer": None},
        }

        # --- Main loop timer ---
        self.timer = self.create_timer(0.5, self.state_loop)  # 2Hz control loop

        self.reset_timer = None

        # Initialize Arduino servo to receive position
        self.safe_send("arm_servo", "3", duration=1.0)
        self.dump_safe_pos = False


        self.get_logger().info("🧭 Controller node started (with device concurrency protection)")

    # ============================================================
    # =================== MAIN LOOP ===============================
    # ============================================================

    def state_loop(self):
        """Main control loop."""
        self.state_pub.publish(String(data=self.current_state))

        if self.current_state == 'idle' or self.current_state == "sys_handle_kernel_change_detect" or self.current_state == "sys_handle_kernel_single_detect" : 
            # self.get_logger().info(" control state loop skipped")
            return  # do nothing when idle
        
        elif self.kernel_state == 'change_detected': 
            return

        elif self.kernel_state == 'detect_pop': 
            return
                
        if self.kernel_state == 'initializing':
            self.handle_initializing()

        elif self.kernel_state == 'wait_for_kernel':
            self.wait_for_kernel()

        elif self.kernel_state == 'single_kernel_detected':
            self.single_kernel_detected()

        elif self.kernel_state == 'excess_kernel_detected':
            self.dump_and_reset()
            # self.dump_and_reset()
        elif self.kernel_state == 'pop_done': 
            self.dump_and_reset()


        else:
            self.get_logger().warn(f"Unknown kernel_state: {self.kernel_state}")

    # ============================================================
    # ================== STATE HANDLERS ==========================
    # ============================================================

    def handle_initializing(self):
        self.get_logger().debug("Initializing — waiting for vision node to stabilize.")

    def wait_for_kernel(self):
        """Run the stepper until a kernel is detected."""
        self.get_logger().info("No kernel yet — running stepper...")
        self.direct_send("singulator", "1")  # immediate motor start

    def change_detected(self):
        """Handle what happens after a single kernel is detected."""
        
        if self.current_state == "sys_handle_kernel_change_detect" : return
        self.current_state = "sys_handle_kernel_change_detect"
        self.get_logger().info("✅ Change Detected! Stopping stepper, counting kernels 1 sec later...")
        # Stop stepper immediately (no lock)
        self.direct_send("singulator", "2")
        self.count_kernels_delay_timer = self.create_timer(1, self.count_kernels_delay)


    def count_kernels_delay(self): 
        self.get_logger().info("... calling count kernels ...")
        self.controller_update.publish(String(data="count_kernels"))
        self.get_logger().info("🔄 Sent count kernel pulse")
        self.controller_update.publish(String(data="idle"))
        self.count_kernels_delay_timer.cancel()
        self.count_kernels_delay_timer = None
        self.current_state = "ready"


    def single_kernel_detected(self):
        """Handle what happens after a single kernel is detected."""
        
        if self.current_state == "sys_handle_kernel_single_detect" : return

        self.current_state = "sys_handle_kernel_single_detect"

        self.get_logger().info("✅ Single kernel detected! Stopping stepper and moving servo.")

        # Stop stepper immediately (no lock)
        self.direct_send("singulator", "2")

        # move to popper
        self.safe_send("arm_servo", "4")

        ####### wait for popper to send signal ##########
        self.current_state = "detect_pop"
        self.controller_update.publish(String(data="detect_pop"))
        self.get_logger().info("🔄 Sent pulse for detect_pop")
        self.controller_update.publish(String(data="idle"))
        #################################################



    
    def dump_and_reset(self): 
        # move to dump position, dump, reset to singulator
        if self.current_state == "sys_handle_kernel_single_detect" : return
        self.current_state = "sys_handle_kernel_change_detect"
        self.get_logger().info("⏱ moving to dump pos in 1 sec...")
        self.move_to_dump_timer = self.create_timer(1, self.move_to_dump)
        self.get_logger().info("⏱ dumping in 1 sec...")
        self.dump_timer = self.create_timer(2, self.dump)
        self.reset_dump_timer = self.create_timer(3, self.reset_dump)
        self.reset_to_singulator_timer = self.create_timer(5, self.reset_to_singulator)

    def move_to_dump(self): 
        self.safe_send("arm_servo", "0")  # move net to dump position
        self.move_to_dump_timer.cancel()
        self.move_to_dump_timer = None

    def dump(self):
        """Runs after 3 seconds to continue the kernel sequence."""
        if self.dump_safe_pos == False : 
            self.get_logger().error("NOT DUMPING -> dump pos not safe")
        else : 
            self.safe_send("dump_servo", "5")  # dump the kernel
        # self.get_logger().info("📨 Sent dump command '5' to dump_servo.")
        self.dump_timer.cancel()
        self.dump_timer = None


    def reset_dump(self): 
        self.safe_send("dump_servo", "6")  # move net to recieve position
        self.reset_dump_timer.cancel()
        self.reset_dump_timer = None

    def reset_to_singulator(self):
        self.safe_send("arm_servo", "3")   # reset arm to singulator position
        self.reset_to_singulator_timer.cancel()
        self.reset_to_singulator_timer = None

        self.get_logger().info("Resetting to wait_for_kernel state.")
        self.kernel_state = "wait_for_kernel"

        self.controller_update.publish(String(data="reset"))
        self.get_logger().info("🔄 Sent reset pulse")
        self.controller_update.publish(String(data="idle"))
        
        self.current_state = "ready"

    def idle(self):
        self.current_state = "idle"

    # ============================================================
    # ================ DEVICE CONCURRENCY LAYER ==================
    # ============================================================

    def safe_send(self, device: str, cmd: str, duration: float = None):
        """
        Safely send an Arduino command to a specific device.
        Prevents overlapping control on the same device.
        """
        if device not in self.device_states:
            self.get_logger().error(f"❌ Unknown device '{device}' for command {cmd}")
            return

        dev = self.device_states[device]

        if dev["busy"]:
            self.get_logger().warn(f"⚠ {device} is busy — ignoring command {cmd}")
            return

        last_cmd = dev.get("last_cmd")

        if last_cmd == cmd:
            return

        # Lock and mark busy
        with dev["lock"]:
            dev["busy"] = True
            self.send_arduino_command(cmd)
            self.get_logger().info(f"✅ safe send command {cmd} to {device}")

            if device == "arm_servo" : 
                if cmd == "0" : self.dump_safe_pos = True
                else: self.dump_safe_pos = False

                self.get_logger().info(f"device: {device}, command: {cmd}: dump safe pos: {self.dump_safe_pos}")


            if duration:
                # Single-shot timer to release device
                def release_and_cancel():
                    self.release_device(device)
                    if "timer" in dev and dev["timer"] is not None:
                        dev["timer"].cancel()
                        dev["timer"] = None

                dev["timer"] = self.create_timer(duration, release_and_cancel)
            else:
                self.release_device(device)


    def release_device(self, device: str):
        """Mark device as idle (after a timer or action completes)."""
        if device in self.device_states:
            dev = self.device_states[device]

            # Cancel any running timer to prevent double release
            if "timer" in dev and dev["timer"] is not None:
                dev["timer"].cancel()
                dev["timer"] = None

            dev["busy"] = False
            self.get_logger().info(f"🔓 {device} released (idle again)")


    def direct_send(self, device: str, cmd: str):
            """
            Immediately send a command to a device (non-blocking, no lock).
            Used for time-critical motors like the singulator.
            Only sends if the command differs from the previous one.
            """
            if device not in self.device_states:
                self.get_logger().error(f"❌ Unknown device '{device}' for command {cmd}")
                return

            dev = self.device_states[device]
            last_cmd = dev.get("last_cmd")

            if last_cmd == cmd:
                self.get_logger().debug(f"↩️ Skipping duplicate command '{cmd}' for {device}")
                return
            
            # Update last command and send immediately
            dev["last_cmd"] = cmd
            self.send_arduino_command(cmd)
            self.get_logger().info(f"⚡ Direct send: new command '{cmd}' → {device} (no lock, immediate)")

    # ============================================================
    # ================= CALLBACKS & HELPERS ======================
    # ============================================================

    def kernel_state_callback(self, msg: String):
        """Receives kernel state from vision node."""
        new_state = msg.data
        if new_state != self.kernel_state:
            self.get_logger().info(f"🧩 Vision state changed: {self.kernel_state} → {new_state}")
        self.kernel_state = new_state

        # Trigger corresponding handler
        if new_state == "change_detected" : 
            self.change_detected()

    def send_arduino_command(self, cmd: str):
        """Publish to Arduino interface node."""
        self.arduino_cmd_pub.publish(String(data=cmd))

    def destroy_node(self):
        """Clean up properly."""
        self.get_logger().info("Shutting down controller...")
        super().destroy_node()


# ============================================================
# ======================== MAIN ===============================
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
