import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports  # Add this explicit import

import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as FigureCanvasTkinter  # FIXED
from matplotlib.figure import Figure
import matplotlib.animation as animation
from collections import deque
import re
import queue

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AK80-8 Motor Controller")
        self.root.geometry("1200x800")
        
        # Serial communication
        self.serial_port = None
        self.is_connected = False
        self.reading_thread = None
        self.stop_reading = False
        
        # Data storage for plotting
        self.max_data_points = 200
        self.timestamps = deque(maxlen=self.max_data_points)
        self.position_data = deque(maxlen=self.max_data_points)
        self.velocity_data = deque(maxlen=self.max_data_points)
        self.torque_data = deque(maxlen=self.max_data_points)
        self.temp_data = deque(maxlen=self.max_data_points)
        
        # Thread-safe queue for data updates
        self.data_queue = queue.Queue()
        
        # Motor status
        self.motor_enabled = False
        self.control_enabled = False
        
        self.setup_gui()
        self.setup_plots()
        
        # Start data processing timer
        self.root.after(50, self.process_data_queue)
        
    def setup_gui(self):
        # Create main frames
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Connection controls
        conn_frame = ttk.LabelFrame(self.control_frame, text="Connection")
        conn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(conn_frame, text="Serial Port:").grid(row=0, column=0, sticky=tk.W)

        available_ports = [port.device for port in serial.tools.list_ports.comports()]

        if not available_ports:
            available_ports = ["/dev/ttyUSB0"]  # Default if none found

            # Refresh button to re-scan ports
        self.refresh_btn = ttk.Button(conn_frame, text="refresh", command=self.refresh_ports, width=3)
        self.refresh_btn.grid(row=0, column=2, padx=2)



        self.port_var = tk.StringVar(value="/dev/ttyUSB0")  # Change default as needed
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, 
                                  values=available_ports, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)

        
        ttk.Label(conn_frame, text="Baud Rate:").grid(row=1, column=0, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_entry = ttk.Entry(conn_frame, textvariable=self.baud_var, width=10)
        self.baud_entry.grid(row=1, column=1, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=1, column=2, padx=5)
        
        # Motor control
        motor_frame = ttk.LabelFrame(self.control_frame, text="Motor Control")
        motor_frame.pack(fill=tk.X, pady=5)
        
        self.enable_btn = ttk.Button(motor_frame, text="Enable MIT Mode", 
                                   command=self.toggle_motor, state=tk.DISABLED)
        self.enable_btn.pack(pady=5)
        
        self.zero_btn = ttk.Button(motor_frame, text="Set Zero Position", 
                                 command=self.set_zero, state=tk.DISABLED)
        self.zero_btn.pack(pady=2)
        
        self.stop_btn = ttk.Button(motor_frame, text="STOP", 
                                 command=self.emergency_stop, state=tk.DISABLED,
                                 style="Danger.TButton")
        self.stop_btn.pack(pady=5)
        
        # Position control
        pos_frame = ttk.LabelFrame(self.control_frame, text="Position Control")
        pos_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(pos_frame, text="Target Angle (degrees):").pack()
        self.angle_var = tk.DoubleVar(value=0.0)
        self.angle_scale = ttk.Scale(pos_frame, from_=-360, to=360, 
                                   orient=tk.HORIZONTAL, variable=self.angle_var,
                                   length=200)
        self.angle_scale.pack()
        
        self.angle_entry = ttk.Entry(pos_frame, textvariable=self.angle_var, width=10)
        self.angle_entry.pack(pady=2)
        
        self.set_angle_btn = ttk.Button(pos_frame, text="Set Target Angle", 
                                      command=self.set_target_angle, state=tk.DISABLED)
        self.set_angle_btn.pack(pady=2)
        
        # Control parameters
        params_frame = ttk.LabelFrame(self.control_frame, text="Control Parameters")
        params_frame.pack(fill=tk.X, pady=5)
        
        # Max Velocity
        ttk.Label(params_frame, text="Max Velocity (rad/s):").pack()
        self.velocity_var = tk.DoubleVar(value=0.5)
        self.velocity_scale = ttk.Scale(params_frame, from_=0.1, to=5.0, 
                                      orient=tk.HORIZONTAL, variable=self.velocity_var,
                                      length=200)
        self.velocity_scale.pack()
        
        self.velocity_entry = ttk.Entry(params_frame, textvariable=self.velocity_var, width=10)
        self.velocity_entry.pack()
        
        self.set_velocity_btn = ttk.Button(params_frame, text="Set Max Velocity", 
                                         command=self.set_max_velocity, state=tk.DISABLED)
        self.set_velocity_btn.pack(pady=2)
        
        # Position Gain (Kp)
        ttk.Label(params_frame, text="Position Gain (Kp):").pack()
        self.kp_var = tk.DoubleVar(value=2.0)
        self.kp_scale = ttk.Scale(params_frame, from_=0.1, to=20.0, 
                                orient=tk.HORIZONTAL, variable=self.kp_var,
                                length=200)
        self.kp_scale.pack()
        
        self.kp_entry = ttk.Entry(params_frame, textvariable=self.kp_var, width=10)
        self.kp_entry.pack()
        
        self.set_kp_btn = ttk.Button(params_frame, text="Set Kp", 
                                   command=self.set_kp, state=tk.DISABLED)
        self.set_kp_btn.pack(pady=2)
        
        # Derivative Gain (Kd)
        ttk.Label(params_frame, text="Derivative Gain (Kd):").pack()
        self.kd_var = tk.DoubleVar(value=0.2)
        self.kd_scale = ttk.Scale(params_frame, from_=0.0, to=5.0, 
                                orient=tk.HORIZONTAL, variable=self.kd_var,
                                length=200)
        self.kd_scale.pack()
        
        self.kd_entry = ttk.Entry(params_frame, textvariable=self.kd_var, width=10)
        self.kd_entry.pack()
        
        self.set_kd_btn = ttk.Button(params_frame, text="Set Kd", 
                                   command=self.set_kd, state=tk.DISABLED)
        self.set_kd_btn.pack(pady=2)
        
        # Status display
        status_frame = ttk.LabelFrame(self.control_frame, text="Motor Status")
        status_frame.pack(fill=tk.X, pady=5)
        
        self.current_pos_label = ttk.Label(status_frame, text="Position: -- °")
        self.current_pos_label.pack()
        
        self.current_vel_label = ttk.Label(status_frame, text="Velocity: -- rad/s")
        self.current_vel_label.pack()
        
        self.current_torque_label = ttk.Label(status_frame, text="Torque: -- A")
        self.current_torque_label.pack()
        
        self.current_temp_label = ttk.Label(status_frame, text="Temperature: -- °C")
        self.current_temp_label.pack()
        
        # Raw command entry
        cmd_frame = ttk.LabelFrame(self.control_frame, text="Direct Commands")
        cmd_frame.pack(fill=tk.X, pady=5)
        
        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var)
        self.cmd_entry.pack(fill=tk.X)
        self.cmd_entry.bind('<Return>', lambda e: self.send_raw_command())
        
        self.send_cmd_btn = ttk.Button(cmd_frame, text="Send Command", 
                                     command=self.send_raw_command, state=tk.DISABLED)
        self.send_cmd_btn.pack(pady=2)

    def refresh_ports(self):
        available_ports = [port.device for port in serial.tools.list_ports.comports()]
        if not available_ports:
            available_ports = ["/dev/ttyUSB0", "/dev/ttyACM0"]  # Common defaults
        
        self.port_combo['values'] = available_ports
        if available_ports:
            self.port_var.set(available_ports[0])

            
    def setup_plots(self):
        # Create matplotlib figure with subplots
        self.fig = Figure(figsize=(10, 8), dpi=100)
        
        # Position plot
        self.ax1 = self.fig.add_subplot(2, 2, 1)
        self.ax1.set_title("Position (degrees)")
        self.ax1.set_ylabel("Angle (°)")
        self.ax1.grid(True)
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=2)
        
        # Velocity plot
        self.ax2 = self.fig.add_subplot(2, 2, 2)
        self.ax2.set_title("Velocity (rad/s)")
        self.ax2.set_ylabel("Velocity (rad/s)")
        self.ax2.grid(True)
        self.line2, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        # Torque plot
        self.ax3 = self.fig.add_subplot(2, 2, 3)
        self.ax3.set_title("Torque (A)")
        self.ax3.set_ylabel("Current (A)")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.grid(True)
        self.line3, = self.ax3.plot([], [], 'r-', linewidth=2)
        
        # Temperature plot
        self.ax4 = self.fig.add_subplot(2, 2, 4)
        self.ax4.set_title("Temperature (°C)")
        self.ax4.set_ylabel("Temperature (°C)")
        self.ax4.set_xlabel("Time (s)")
        self.ax4.grid(True)
        self.line4, = self.ax4.plot([], [], 'orange', linewidth=2)
        
        # Embed plots in tkinter
        self.canvas = FigureCanvasTkinter(self.fig, self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.animate, interval=100, blit=False)
        
    def animate(self, frame):
        if len(self.timestamps) > 1:
            # Convert timestamps to relative seconds
            time_data = [(t - self.timestamps[0]) for t in self.timestamps]
            
            # Update position plot (convert  radian to degrees)
            pos_degrees = [p * 180.0 / 3.14159 for p in self.position_data]
            self.line1.set_data(time_data, pos_degrees)
            self.ax1.relim()
            self.ax1.autoscale_view()
            
            # Update velocity plot
            self.line2.set_data(time_data, list(self.velocity_data))
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            # Update torque plot
            self.line3.set_data(time_data, list(self.torque_data))
            self.ax3.relim()
            self.ax3.autoscale_view()
            
            # Update temperature plot
            self.line4.set_data(time_data, list(self.temp_data))
            self.ax4.relim()
            self.ax4.autoscale_view()
            
        return self.line1, self.line2, self.line3, self.line4
        
    def toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.port_var.get()
                baud = int(self.baud_var.get())
                self.serial_port = serial.Serial(port, baud, timeout=0.1)
                
                self.is_connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Connected", foreground="green")
                
                # Enable controls
                self.enable_controls(True)
                
                # Start reading thread
                self.stop_reading = False
                self.reading_thread = threading.Thread(target=self.read_serial_data)
                self.reading_thread.daemon = True
                self.reading_thread.start()
                
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
        else:
            self.disconnect()
            
    def disconnect(self):
        self.is_connected = False
        self.stop_reading = True
        
        if self.serial_port:
            self.serial_port.close()
            
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        
        # Disable controls
        self.enable_controls(False)
        
    def enable_controls(self, enable):
        state = tk.NORMAL if enable else tk.DISABLED
        
        self.enable_btn.config(state=state)
        self.zero_btn.config(state=state)
        self.stop_btn.config(state=state)
        self.set_angle_btn.config(state=state)
        self.set_velocity_btn.config(state=state)
        self.set_kp_btn.config(state=state)
        self.set_kd_btn.config(state=state)
        self.send_cmd_btn.config(state=state)
        
    def send_command(self, command):
        if self.serial_port and self.is_connected:
            try:
                self.serial_port.write((command + '\r\n').encode())
                print(f"Sent: {command}")
            except Exception as e:
                messagebox.showerror("Communication Error", f"Failed to send command: {str(e)}")
                
    def toggle_motor(self):
        if not self.motor_enabled:
            self.send_command('m')
            self.enable_btn.config(text="Disable Motor")
            self.motor_enabled = True
        else:
            self.send_command('e')
            self.enable_btn.config(text="Enable MIT Mode")
            self.motor_enabled = False
            self.control_enabled = False
            
    def set_zero(self):
        self.send_command('z')
        
    def emergency_stop(self):
        self.send_command('stop')
        
    def set_target_angle(self):
        angle = self.angle_var.get()
        self.send_command(f'a {angle}')
        
    def set_max_velocity(self):
        velocity = self.velocity_var.get()
        self.send_command(f'v {velocity}')
        
    def set_kp(self):
        kp = self.kp_var.get()
        self.send_command(f'kp {kp}')
        
    def set_kd(self):
        kd = self.kd_var.get()
        self.send_command(f'kd {kd}')
        
    def send_raw_command(self):
        cmd = self.cmd_var.get().strip()
        if cmd:
            self.send_command(cmd)
            self.cmd_var.set("")
            
    def read_serial_data(self):
        while not self.stop_reading and self.is_connected:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.parse_motor_data(line)
            except Exception as e:
                print(f"Serial read error: {e}")
                
            time.sleep(0.01)
            
    def parse_motor_data(self, line):
        # Parse motor response data
        # Format: "Motor Response - ID:1, Pos:1.234 rad, Vel:0.50 rad/s, Curr:2.1 A, Temp:25.0°C"
        motor_pattern = r'Motor Response - ID:(\d+), Pos:([-+]?\d*\.?\d+) rad, Vel:([-+]?\d*\.?\d+) rad/s, Curr:([-+]?\d*\.?\d+) A, Temp:([-+]?\d*\.?\d+)'
        
        match = re.search(motor_pattern, line)
        if match:
            try:
                pos_rad = float(match.group(2))
                vel_rads = float(match.group(3))
                current_a = float(match.group(4))
                temp_c = float(match.group(5))
                
                # Add to data queue for thread-safe GUI updates
                data = {
                    'timestamp': time.time(),
                    'position': pos_rad,
                    'velocity': vel_rads,
                    'torque': current_a,
                    'temperature': temp_c
                }
                
                self.data_queue.put(data)
                
            except ValueError:
                pass  # Skip malformed data
                
    def process_data_queue(self):
        try:
            while not self.data_queue.empty():
                data = self.data_queue.get_nowait()
                
                # Add to plotting data
                self.timestamps.append(data['timestamp'])
                self.position_data.append(data['position'])
                self.velocity_data.append(data['velocity'])
                self.torque_data.append(data['torque'])
                self.temp_data.append(data['temperature'])
                
                # Update status labels
                pos_deg = data['position'] * 180.0 / 3.14159
                self.current_pos_label.config(text=f"Position: {pos_deg:.2f} °")
                self.current_vel_label.config(text=f"Velocity: {data['velocity']:.3f} rad/s")
                self.current_torque_label.config(text=f"Torque: {data['torque']:.2f} A")
                self.current_temp_label.config(text=f"Temperature: {data['temperature']:.1f} °C")
                
        except queue.Empty:
            pass
            
        # Schedule next check
        self.root.after(50, self.process_data_queue)

def main():
    root = tk.Tk()
    
    # Configure style for danger button
    style = ttk.Style()
    style.configure("Danger.TButton", foreground="red")
    
    app = MotorControlGUI(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        if app.is_connected:
            app.disconnect()

if __name__ == "__main__":
    main()
