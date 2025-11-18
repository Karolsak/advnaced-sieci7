"""
Advanced Train Energy Calculation and Multi-Physics Simulation Tool
Features: Dynamic simulation, ODE solvers, electromagnetic-thermal-mechanical coupling,
economic analysis, and comprehensive visualization
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import threading
import time
from datetime import datetime

class TrainEnergySimulator:
    """Advanced train energy and multi-physics simulator"""

    def __init__(self):
        # Default parameters
        self.mass = 400  # tonnes
        self.distance = 10  # km
        self.gradient = 2  # %
        self.v_initial = 40  # km/h
        self.v_final = 20  # km/h
        self.resistance = 50  # N/t
        self.rotational_inertia = 10  # %
        self.efficiency = 72  # %

        # Multi-physics parameters
        self.motor_voltage = 1500  # V (RMS)
        self.motor_current = 0  # A (RMS)
        self.motor_power_factor = 0.85
        self.ambient_temp = 25  # °C
        self.motor_temp = 25  # °C
        self.thermal_resistance = 0.5  # °C/W
        self.thermal_capacitance = 5000  # J/°C
        self.shaft_torque = 0  # Nm
        self.bearing_load = 0  # N

        # Loss components
        self.copper_losses = 0
        self.iron_losses = 0
        self.mechanical_losses = 0
        self.stray_losses = 0

        # Simulation control
        self.simulation_running = False
        self.simulation_paused = False
        self.time_data = []
        self.speed_data = []
        self.power_data = []
        self.energy_data = []
        self.temp_data = []
        self.torque_data = []
        self.efficiency_data = []

        # Economic parameters
        self.electricity_cost = 0.12  # $/kWh
        self.maintenance_cost = 0.05  # $/km
        self.depreciation_rate = 0.1  # per year

    def calculate_train_energy(self):
        """Calculate power and energy returned to the line"""
        # Convert units
        m = self.mass * 1000  # kg
        d = self.distance * 1000  # m
        v1 = self.v_initial / 3.6  # m/s
        v2 = self.v_final / 3.6  # m/s
        R = self.resistance  # N/t
        grad = self.gradient / 100  # fraction
        rot_factor = 1 + (self.rotational_inertia / 100)
        eta = self.efficiency / 100

        # Kinetic energy change (with rotational inertia)
        delta_ke = 0.5 * m * rot_factor * (v1**2 - v2**2)  # J

        # Potential energy change (downhill is negative)
        delta_pe = -m * 9.81 * d * grad  # J (negative for downhill)

        # Resistance work
        resistance_force = R * self.mass  # N
        resistance_work = resistance_force * d  # J

        # Total energy available (kinetic + potential - resistance)
        total_energy = delta_ke + delta_pe - resistance_work  # J

        # Energy returned to line (accounting for efficiency)
        energy_returned = total_energy * eta  # J
        energy_returned_kwh = energy_returned / 3.6e6  # kWh

        # Calculate average power
        avg_velocity = (v1 + v2) / 2  # m/s
        time_taken = d / avg_velocity if avg_velocity > 0 else 0  # s
        power_returned = energy_returned / time_taken if time_taken > 0 else 0  # W
        power_returned_kw = power_returned / 1000  # kW

        return {
            'power_kw': power_returned_kw,
            'energy_kwh': energy_returned_kwh,
            'time_s': time_taken,
            'delta_ke': delta_ke / 1e6,  # MJ
            'delta_pe': delta_pe / 1e6,  # MJ
            'resistance_work': resistance_work / 1e6,  # MJ
            'total_energy': total_energy / 1e6  # MJ
        }

    def calculate_motor_parameters(self, speed_kmh, power_kw):
        """Calculate motor electrical parameters (RMS values)"""
        # Motor speed in rpm (assuming gear ratio)
        gear_ratio = 15
        motor_rpm = (speed_kmh / 3.6) * 60 * gear_ratio / (2 * np.pi * 0.5)

        # Motor torque
        speed_rads = speed_kmh / 3.6 * gear_ratio / 0.5
        self.shaft_torque = (power_kw * 1000) / speed_rads if speed_rads > 0 else 0

        # Motor current (RMS) - simplified model
        apparent_power = abs(power_kw) / self.motor_power_factor  # kVA
        self.motor_current = (apparent_power * 1000) / (self.motor_voltage * np.sqrt(3))

        # Bearing load estimation
        self.bearing_load = self.mass * 9.81 * 1000 / 4  # N (distributed over 4 axles)

        return motor_rpm

    def calculate_losses(self, current_rms, speed_kmh):
        """Calculate detailed loss breakdown"""
        # Copper losses (I²R losses)
        resistance_per_phase = 0.1  # Ohm
        self.copper_losses = 3 * (current_rms ** 2) * resistance_per_phase / 1000  # kW

        # Iron losses (core losses) - frequency dependent
        frequency = speed_kmh / 3.6 * 2  # Simplified
        self.iron_losses = 0.5 + 0.001 * frequency ** 2  # kW

        # Mechanical losses (friction and windage)
        self.mechanical_losses = 0.2 + 0.0001 * speed_kmh ** 2  # kW

        # Stray load losses
        self.stray_losses = 0.01 * abs(self.motor_current) * self.motor_voltage / 1000  # kW

        total_losses = (self.copper_losses + self.iron_losses +
                       self.mechanical_losses + self.stray_losses)

        return total_losses

    def thermal_model(self, t, T, power_loss):
        """Thermal differential equation: dT/dt = (P_loss - (T-T_amb)/R_th) / C_th"""
        T_amb = self.ambient_temp
        R_th = self.thermal_resistance
        C_th = self.thermal_capacitance

        dT_dt = (power_loss * 1000 - (T - T_amb) / R_th) / C_th
        return dT_dt

    def train_dynamics_ode(self, t, y, method='rk45'):
        """
        Train dynamics ODE system
        y[0] = position (m)
        y[1] = velocity (m/s)
        y[2] = temperature (°C)
        """
        position, velocity, temperature = y

        # Convert to km/h for calculations
        speed_kmh = velocity * 3.6

        # Forces
        m = self.mass * 1000  # kg
        grad = self.gradient / 100
        rot_factor = 1 + (self.rotational_inertia / 100)

        # Gravitational force (negative for downhill)
        F_gravity = -m * 9.81 * grad

        # Resistance force
        F_resistance = -self.resistance * self.mass

        # Braking force (to achieve speed reduction)
        target_decel = -0.15  # m/s²
        F_brake = m * rot_factor * target_decel if velocity > self.v_final/3.6 else 0

        # Total force
        F_total = F_gravity + F_resistance + F_brake

        # Acceleration
        acceleration = F_total / (m * rot_factor)

        # Limit minimum speed
        if velocity <= self.v_final / 3.6:
            acceleration = 0
            velocity = self.v_final / 3.6

        # Calculate power
        power_kw = F_total * velocity / 1000

        # Calculate motor parameters and losses
        self.calculate_motor_parameters(speed_kmh, power_kw)
        total_losses = self.calculate_losses(self.motor_current, speed_kmh)

        # Thermal dynamics
        dT_dt = self.thermal_model(t, temperature, total_losses)

        # State derivatives
        dy = np.zeros(3)
        dy[0] = velocity  # dx/dt = v
        dy[1] = acceleration  # dv/dt = a
        dy[2] = dT_dt  # dT/dt

        return dy

    def solve_ode_rk45(self, t_span, y0, dt=0.1):
        """Solve ODE using RK45 (Runge-Kutta 4-5 method)"""
        sol = solve_ivp(
            lambda t, y: self.train_dynamics_ode(t, y, method='rk45'),
            t_span,
            y0,
            method='RK45',
            max_step=dt,
            dense_output=True
        )
        return sol

    def solve_ode_euler(self, t_span, y0, dt=0.1):
        """Solve ODE using Euler method"""
        t0, tf = t_span
        t = np.arange(t0, tf, dt)
        n = len(t)
        y = np.zeros((n, len(y0)))
        y[0] = y0

        for i in range(1, n):
            dy = self.train_dynamics_ode(t[i-1], y[i-1], method='euler')
            y[i] = y[i-1] + dt * dy

        return t, y

    def check_thermal_derating(self, temperature):
        """Check if thermal derating is required"""
        max_temp = 120  # °C
        derating_temp = 100  # °C

        if temperature > max_temp:
            return 0  # Complete shutdown
        elif temperature > derating_temp:
            # Linear derating
            derating_factor = (max_temp - temperature) / (max_temp - derating_temp)
            return max(0, derating_factor)
        else:
            return 1  # No derating

    def calculate_economic_metrics(self, energy_kwh, distance_km, time_hours):
        """Calculate economic analysis metrics"""
        # Energy cost
        energy_cost = abs(energy_kwh) * self.electricity_cost

        # Maintenance cost
        maint_cost = distance_km * self.maintenance_cost

        # Operating cost per km
        total_cost = energy_cost + maint_cost
        cost_per_km = total_cost / distance_km if distance_km > 0 else 0

        # Energy savings (regenerative braking)
        if energy_kwh < 0:  # Energy returned
            savings = abs(energy_kwh) * self.electricity_cost
        else:
            savings = 0

        # Efficiency metrics
        energy_efficiency = abs(energy_kwh) / (distance_km * self.mass) if distance_km > 0 else 0

        return {
            'energy_cost': energy_cost,
            'maintenance_cost': maint_cost,
            'total_cost': total_cost,
            'cost_per_km': cost_per_km,
            'savings': savings,
            'energy_efficiency': energy_efficiency
        }


class TrainSimulatorGUI:
    """Advanced Tkinter GUI for train energy simulator"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Train Energy & Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Simulator instance
        self.simulator = TrainEnergySimulator()

        # Simulation thread
        self.sim_thread = None

        # Configure grid weight for auto-scaling
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Create main container
        self.main_container = ttk.Frame(self.root)
        self.main_container.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        self.main_container.grid_rowconfigure(1, weight=1)
        self.main_container.grid_columnconfigure(0, weight=1)

        # Create menu bar
        self.create_menu()

        # Create control panel
        self.create_control_panel()

        # Create notebook (tabs)
        self.notebook = ttk.Notebook(self.main_container)
        self.notebook.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.create_parameters_tab()
        self.create_simulation_tab()
        self.create_multiphysics_tab()
        self.create_economic_tab()
        self.create_results_tab()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        self.status_bar = ttk.Label(self.main_container, textvariable=self.status_var,
                                    relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.grid(row=2, column=0, sticky='ew', padx=5, pady=2)

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Results", command=self.save_results)
        file_menu.add_command(label="Load Parameters", command=self.load_parameters)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Simulation menu
        sim_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        sim_menu.add_command(label="Quick Calculate", command=self.quick_calculate)
        sim_menu.add_command(label="Run Dynamic Simulation", command=self.start_simulation)
        sim_menu.add_command(label="Stop Simulation", command=self.stop_simulation)
        sim_menu.add_command(label="Reset", command=self.reset_simulation)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
        help_menu.add_command(label="Documentation", command=self.show_documentation)

    def create_control_panel(self):
        """Create main control panel with buttons"""
        control_frame = ttk.LabelFrame(self.main_container, text="Simulation Control", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        # Buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(side=tk.LEFT, padx=5)

        self.start_btn = ttk.Button(btn_frame, text="▶ Start", command=self.start_simulation,
                                    style='Success.TButton', width=12)
        self.start_btn.pack(side=tk.LEFT, padx=2)

        self.pause_btn = ttk.Button(btn_frame, text="⏸ Pause", command=self.pause_simulation,
                                    width=12, state='disabled')
        self.pause_btn.pack(side=tk.LEFT, padx=2)

        self.stop_btn = ttk.Button(btn_frame, text="⏹ Stop", command=self.stop_simulation,
                                   width=12, state='disabled')
        self.stop_btn.pack(side=tk.LEFT, padx=2)

        self.reset_btn = ttk.Button(btn_frame, text="↺ Reset", command=self.reset_simulation,
                                    width=12)
        self.reset_btn.pack(side=tk.LEFT, padx=2)

        self.calc_btn = ttk.Button(btn_frame, text="⚡ Quick Calc", command=self.quick_calculate,
                                   width=12)
        self.calc_btn.pack(side=tk.LEFT, padx=2)

        # ODE Method selection
        method_frame = ttk.Frame(control_frame)
        method_frame.pack(side=tk.LEFT, padx=20)

        ttk.Label(method_frame, text="ODE Solver:").pack(side=tk.LEFT, padx=5)
        self.ode_method = tk.StringVar(value='RK45')
        ode_combo = ttk.Combobox(method_frame, textvariable=self.ode_method,
                                 values=['RK45', 'Euler'], width=10, state='readonly')
        ode_combo.pack(side=tk.LEFT, padx=5)

        # Time step
        ttk.Label(method_frame, text="Time Step (s):").pack(side=tk.LEFT, padx=5)
        self.time_step_var = tk.DoubleVar(value=0.1)
        time_step_spin = ttk.Spinbox(method_frame, from_=0.01, to=1.0, increment=0.01,
                                     textvariable=self.time_step_var, width=8)
        time_step_spin.pack(side=tk.LEFT, padx=5)

    def create_parameters_tab(self):
        """Create parameters input tab with sliders"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="📊 Input Parameters")

        # Configure grid
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_columnconfigure(1, weight=1)
        tab.grid_rowconfigure(0, weight=1)

        # Left panel - Train parameters
        left_frame = ttk.LabelFrame(tab, text="Train Parameters", padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        self.param_vars = {}
        parameters = [
            ('mass', 'Train Mass (tonnes)', 100, 1000, 400),
            ('distance', 'Distance (km)', 1, 100, 10),
            ('gradient', 'Gradient (%)', -5, 5, 2),
            ('v_initial', 'Initial Speed (km/h)', 0, 200, 40),
            ('v_final', 'Final Speed (km/h)', 0, 200, 20),
            ('resistance', 'Resistance (N/t)', 10, 200, 50),
            ('rotational_inertia', 'Rotational Inertia (%)', 0, 50, 10),
            ('efficiency', 'Efficiency (%)', 50, 100, 72),
        ]

        for i, (key, label, min_val, max_val, default) in enumerate(parameters):
            frame = ttk.Frame(left_frame)
            frame.pack(fill='x', padx=5, pady=5)

            ttk.Label(frame, text=label, width=25).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=default)
            self.param_vars[key] = var

            slider = ttk.Scale(frame, from_=min_val, to=max_val, variable=var,
                              orient='horizontal', length=200)
            slider.pack(side=tk.LEFT, padx=5, fill='x', expand=True)

            entry = ttk.Entry(frame, textvariable=var, width=10)
            entry.pack(side=tk.LEFT, padx=5)

            ttk.Label(frame, text=f"[{min_val}-{max_val}]", width=15).pack(side=tk.LEFT)

        # Right panel - Motor and multi-physics parameters
        right_frame = ttk.LabelFrame(tab, text="Motor & Multi-Physics Parameters", padding=10)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        motor_parameters = [
            ('motor_voltage', 'Motor Voltage RMS (V)', 500, 3000, 1500),
            ('motor_power_factor', 'Power Factor', 0.5, 1.0, 0.85),
            ('ambient_temp', 'Ambient Temperature (°C)', -20, 50, 25),
            ('thermal_resistance', 'Thermal Resistance (°C/W)', 0.1, 2.0, 0.5),
            ('thermal_capacitance', 'Thermal Capacitance (J/°C)', 1000, 10000, 5000),
        ]

        for key, label, min_val, max_val, default in motor_parameters:
            frame = ttk.Frame(right_frame)
            frame.pack(fill='x', padx=5, pady=5)

            ttk.Label(frame, text=label, width=30).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=default)
            self.param_vars[key] = var

            slider = ttk.Scale(frame, from_=min_val, to=max_val, variable=var,
                              orient='horizontal', length=180)
            slider.pack(side=tk.LEFT, padx=5, fill='x', expand=True)

            entry = ttk.Entry(frame, textvariable=var, width=10)
            entry.pack(side=tk.LEFT, padx=5)

        # Update button
        update_btn = ttk.Button(right_frame, text="Update Parameters",
                               command=self.update_simulator_params)
        update_btn.pack(pady=10)

    def create_simulation_tab(self):
        """Create dynamic simulation visualization tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="🎯 Dynamic Simulation")

        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Create matplotlib figure
        self.sim_fig = Figure(figsize=(12, 8), dpi=100)

        # Create subplots
        self.ax_speed = self.sim_fig.add_subplot(2, 2, 1)
        self.ax_power = self.sim_fig.add_subplot(2, 2, 2)
        self.ax_energy = self.sim_fig.add_subplot(2, 2, 3)
        self.ax_temp = self.sim_fig.add_subplot(2, 2, 4)

        self.sim_fig.tight_layout(pad=3.0)

        # Create canvas
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, tab)
        self.sim_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Initialize plots
        self.init_simulation_plots()

    def create_multiphysics_tab(self):
        """Create multi-physics analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="⚙️ Multi-Physics")

        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Create figure for multi-physics
        self.mp_fig = Figure(figsize=(12, 8), dpi=100)

        self.ax_losses = self.mp_fig.add_subplot(2, 2, 1)
        self.ax_torque = self.mp_fig.add_subplot(2, 2, 2)
        self.ax_current = self.mp_fig.add_subplot(2, 2, 3)
        self.ax_efficiency = self.mp_fig.add_subplot(2, 2, 4)

        self.mp_fig.tight_layout(pad=3.0)

        self.mp_canvas = FigureCanvasTkAgg(self.mp_fig, tab)
        self.mp_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        self.init_multiphysics_plots()

    def create_economic_tab(self):
        """Create economic analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="💰 Economic Analysis")

        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Economic parameters
        param_frame = ttk.LabelFrame(tab, text="Economic Parameters", padding=10)
        param_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        econ_params = [
            ('electricity_cost', 'Electricity Cost ($/kWh)', 0.05, 0.5, 0.12),
            ('maintenance_cost', 'Maintenance Cost ($/km)', 0.01, 0.2, 0.05),
            ('depreciation_rate', 'Depreciation Rate (per year)', 0.05, 0.3, 0.1),
        ]

        for key, label, min_val, max_val, default in econ_params:
            frame = ttk.Frame(param_frame)
            frame.pack(fill='x', padx=5, pady=3)

            ttk.Label(frame, text=label, width=30).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=default)
            self.param_vars[key] = var

            slider = ttk.Scale(frame, from_=min_val, to=max_val, variable=var,
                              orient='horizontal', length=300)
            slider.pack(side=tk.LEFT, padx=5, fill='x', expand=True)

            entry = ttk.Entry(frame, textvariable=var, width=10)
            entry.pack(side=tk.LEFT, padx=5)

        # Results display
        results_frame = ttk.LabelFrame(tab, text="Economic Results", padding=10)
        results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        # Create text widget for results
        self.econ_text = tk.Text(results_frame, height=20, width=80, font=('Courier', 10))
        self.econ_text.pack(side=tk.LEFT, fill='both', expand=True, padx=5, pady=5)

        scrollbar = ttk.Scrollbar(results_frame, command=self.econ_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill='y')
        self.econ_text.config(yscrollcommand=scrollbar.set)

        # Calculate button
        calc_btn = ttk.Button(results_frame, text="Calculate Economics",
                             command=self.calculate_economics)
        calc_btn.pack(pady=5)

    def create_results_tab(self):
        """Create results summary tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="📈 Results Summary")

        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Create text widget for results
        text_frame = ttk.Frame(tab)
        text_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        text_frame.grid_rowconfigure(0, weight=1)
        text_frame.grid_columnconfigure(0, weight=1)

        self.results_text = tk.Text(text_frame, font=('Courier', 10), wrap='word')
        self.results_text.grid(row=0, column=0, sticky='nsew')

        scrollbar = ttk.Scrollbar(text_frame, command=self.results_text.yview)
        scrollbar.grid(row=0, column=1, sticky='ns')
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Export button
        export_btn = ttk.Button(tab, text="Export Results to File",
                               command=self.export_results)
        export_btn.grid(row=1, column=0, pady=5)

    def init_simulation_plots(self):
        """Initialize simulation plots"""
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (km/h)')
        self.ax_speed.set_title('Train Speed vs Time')
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.set_title('Power vs Time')
        self.ax_power.grid(True, alpha=0.3)

        self.ax_energy.set_xlabel('Time (s)')
        self.ax_energy.set_ylabel('Energy (kWh)')
        self.ax_energy.set_title('Cumulative Energy')
        self.ax_energy.grid(True, alpha=0.3)

        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Motor Temperature')
        self.ax_temp.grid(True, alpha=0.3)

        self.sim_canvas.draw()

    def init_multiphysics_plots(self):
        """Initialize multi-physics plots"""
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Losses (kW)')
        self.ax_losses.set_title('Loss Breakdown')
        self.ax_losses.grid(True, alpha=0.3)

        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (Nm)')
        self.ax_torque.set_title('Shaft Torque')
        self.ax_torque.grid(True, alpha=0.3)

        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current RMS (A)')
        self.ax_current.set_title('Motor Current')
        self.ax_current.grid(True, alpha=0.3)

        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('System Efficiency')
        self.ax_efficiency.grid(True, alpha=0.3)

        self.mp_canvas.draw()

    def update_simulator_params(self):
        """Update simulator parameters from GUI"""
        for key, var in self.param_vars.items():
            if hasattr(self.simulator, key):
                setattr(self.simulator, key, var.get())

        self.status_var.set("Parameters updated successfully")
        messagebox.showinfo("Success", "Parameters updated successfully!")

    def quick_calculate(self):
        """Perform quick energy calculation"""
        self.update_simulator_params()
        results = self.simulator.calculate_train_energy()

        # Display results
        output = "="*60 + "\n"
        output += "QUICK CALCULATION RESULTS\n"
        output += "="*60 + "\n\n"
        output += f"Power Returned to Line: {results['power_kw']:.2f} kW\n"
        output += f"Energy Returned to Line: {results['energy_kwh']:.4f} kWh\n"
        output += f"Time Duration: {results['time_s']:.2f} seconds\n\n"
        output += "Energy Components:\n"
        output += f"  Kinetic Energy Change: {results['delta_ke']:.4f} MJ\n"
        output += f"  Potential Energy Change: {results['delta_pe']:.4f} MJ\n"
        output += f"  Resistance Work: {results['resistance_work']:.4f} MJ\n"
        output += f"  Total Energy Available: {results['total_energy']:.4f} MJ\n"
        output += "="*60 + "\n"

        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(1.0, output)

        self.status_var.set(f"Quick calculation complete: {results['power_kw']:.2f} kW, {results['energy_kwh']:.4f} kWh")

        # Switch to results tab
        self.notebook.select(4)

    def start_simulation(self):
        """Start dynamic simulation"""
        if self.simulator.simulation_running:
            messagebox.showwarning("Warning", "Simulation already running!")
            return

        self.update_simulator_params()

        # Reset data
        self.simulator.time_data = []
        self.simulator.speed_data = []
        self.simulator.power_data = []
        self.simulator.energy_data = []
        self.simulator.temp_data = []
        self.simulator.torque_data = []
        self.simulator.efficiency_data = []

        # Update button states
        self.start_btn.config(state='disabled')
        self.pause_btn.config(state='normal')
        self.stop_btn.config(state='normal')

        self.simulator.simulation_running = True
        self.simulator.simulation_paused = False

        # Start simulation thread
        self.sim_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.sim_thread.start()

        self.status_var.set("Simulation running...")

    def pause_simulation(self):
        """Pause/resume simulation"""
        if self.simulator.simulation_running:
            self.simulator.simulation_paused = not self.simulator.simulation_paused
            if self.simulator.simulation_paused:
                self.pause_btn.config(text="▶ Resume")
                self.status_var.set("Simulation paused")
            else:
                self.pause_btn.config(text="⏸ Pause")
                self.status_var.set("Simulation running...")

    def stop_simulation(self):
        """Stop simulation"""
        self.simulator.simulation_running = False
        self.simulator.simulation_paused = False

        # Update button states
        self.start_btn.config(state='normal')
        self.pause_btn.config(state='disabled', text="⏸ Pause")
        self.stop_btn.config(state='disabled')

        self.status_var.set("Simulation stopped")

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()

        # Clear data
        self.simulator.time_data = []
        self.simulator.speed_data = []
        self.simulator.power_data = []
        self.simulator.energy_data = []
        self.simulator.temp_data = []
        self.simulator.torque_data = []
        self.simulator.efficiency_data = []

        # Clear plots
        self.ax_speed.clear()
        self.ax_power.clear()
        self.ax_energy.clear()
        self.ax_temp.clear()
        self.ax_losses.clear()
        self.ax_torque.clear()
        self.ax_current.clear()
        self.ax_efficiency.clear()

        self.init_simulation_plots()
        self.init_multiphysics_plots()

        self.status_var.set("Simulation reset")

    def run_simulation(self):
        """Run dynamic simulation in separate thread"""
        try:
            # Initial conditions
            y0 = [
                0,  # position (m)
                self.simulator.v_initial / 3.6,  # velocity (m/s)
                self.simulator.ambient_temp  # temperature (°C)
            ]

            # Time span (estimate)
            avg_velocity = (self.simulator.v_initial + self.simulator.v_final) / 2 / 3.6
            total_time = (self.simulator.distance * 1000) / avg_velocity if avg_velocity > 0 else 100
            t_span = (0, total_time)

            dt = self.time_step_var.get()
            method = self.ode_method.get()

            if method == 'RK45':
                # Solve using RK45
                sol = self.simulator.solve_ode_rk45(t_span, y0, dt)
                t_eval = np.linspace(0, total_time, int(total_time / dt))
                y_eval = sol.sol(t_eval).T

                # Process results in real-time fashion
                for i in range(len(t_eval)):
                    if not self.simulator.simulation_running:
                        break

                    while self.simulator.simulation_paused:
                        time.sleep(0.1)
                        if not self.simulator.simulation_running:
                            break

                    t = t_eval[i]
                    position, velocity, temperature = y_eval[i]

                    self.process_simulation_step(t, position, velocity, temperature)

                    # Small delay for visualization
                    time.sleep(0.01)
            else:
                # Euler method
                t, y = self.simulator.solve_ode_euler(t_span, y0, dt)

                for i in range(len(t)):
                    if not self.simulator.simulation_running:
                        break

                    while self.simulator.simulation_paused:
                        time.sleep(0.1)
                        if not self.simulator.simulation_running:
                            break

                    position, velocity, temperature = y[i]
                    self.process_simulation_step(t[i], position, velocity, temperature)

                    time.sleep(0.01)

            # Simulation complete
            if self.simulator.simulation_running:
                self.root.after(0, self.simulation_complete)

        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Error", f"Simulation error: {str(e)}"))
            self.root.after(0, self.stop_simulation)

    def process_simulation_step(self, t, position, velocity, temperature):
        """Process one simulation step"""
        speed_kmh = velocity * 3.6

        # Calculate power
        m = self.simulator.mass * 1000
        grad = self.simulator.gradient / 100
        F_gravity = -m * 9.81 * grad
        F_resistance = -self.simulator.resistance * self.simulator.mass
        power_kw = (F_gravity + F_resistance) * velocity / 1000

        # Calculate motor parameters
        self.simulator.calculate_motor_parameters(speed_kmh, power_kw)
        total_losses = self.simulator.calculate_losses(self.simulator.motor_current, speed_kmh)

        # Calculate efficiency
        if abs(power_kw) > 0.01:
            efficiency = (abs(power_kw) - total_losses) / abs(power_kw) * 100
        else:
            efficiency = 0

        # Check thermal derating
        derating = self.simulator.check_thermal_derating(temperature)

        # Store data
        self.simulator.time_data.append(t)
        self.simulator.speed_data.append(speed_kmh)
        self.simulator.power_data.append(power_kw)

        # Cumulative energy (kWh)
        if len(self.simulator.energy_data) > 0:
            dt = t - self.simulator.time_data[-2] if len(self.simulator.time_data) > 1 else 0.1
            energy_increment = power_kw * dt / 3600  # kWh
            cumulative_energy = self.simulator.energy_data[-1] + energy_increment
        else:
            cumulative_energy = 0

        self.simulator.energy_data.append(cumulative_energy)
        self.simulator.temp_data.append(temperature)
        self.simulator.torque_data.append(self.simulator.shaft_torque)
        self.simulator.efficiency_data.append(efficiency)

        # Update plots every 10 steps
        if len(self.simulator.time_data) % 10 == 0:
            self.root.after(0, self.update_plots)

    def update_plots(self):
        """Update all plots with current data"""
        if len(self.simulator.time_data) < 2:
            return

        # Update simulation plots
        self.ax_speed.clear()
        self.ax_speed.plot(self.simulator.time_data, self.simulator.speed_data, 'b-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (km/h)')
        self.ax_speed.set_title('Train Speed vs Time')
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_power.clear()
        self.ax_power.plot(self.simulator.time_data, self.simulator.power_data, 'r-', linewidth=2)
        self.ax_power.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.set_title('Power vs Time')
        self.ax_power.grid(True, alpha=0.3)

        self.ax_energy.clear()
        self.ax_energy.plot(self.simulator.time_data, self.simulator.energy_data, 'g-', linewidth=2)
        self.ax_energy.set_xlabel('Time (s)')
        self.ax_energy.set_ylabel('Energy (kWh)')
        self.ax_energy.set_title('Cumulative Energy')
        self.ax_energy.grid(True, alpha=0.3)

        self.ax_temp.clear()
        self.ax_temp.plot(self.simulator.time_data, self.simulator.temp_data, 'orange', linewidth=2)
        self.ax_temp.axhline(y=100, color='r', linestyle='--', label='Derating Temp', alpha=0.5)
        self.ax_temp.axhline(y=120, color='darkred', linestyle='--', label='Max Temp', alpha=0.5)
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Motor Temperature')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        self.sim_canvas.draw()

        # Update multi-physics plots
        if len(self.simulator.time_data) > 1:
            # Calculate loss components over time
            copper_losses_arr = []
            iron_losses_arr = []
            mech_losses_arr = []
            stray_losses_arr = []
            current_arr = []

            for i, t in enumerate(self.simulator.time_data):
                speed = self.simulator.speed_data[i]
                # Re-calculate losses for this time step
                motor_current = abs(self.simulator.power_data[i]) * 10  # Simplified
                self.simulator.calculate_losses(motor_current, speed)

                copper_losses_arr.append(self.simulator.copper_losses)
                iron_losses_arr.append(self.simulator.iron_losses)
                mech_losses_arr.append(self.simulator.mechanical_losses)
                stray_losses_arr.append(self.simulator.stray_losses)
                current_arr.append(motor_current)

            self.ax_losses.clear()
            self.ax_losses.plot(self.simulator.time_data, copper_losses_arr, label='Copper Losses', linewidth=2)
            self.ax_losses.plot(self.simulator.time_data, iron_losses_arr, label='Iron Losses', linewidth=2)
            self.ax_losses.plot(self.simulator.time_data, mech_losses_arr, label='Mechanical Losses', linewidth=2)
            self.ax_losses.plot(self.simulator.time_data, stray_losses_arr, label='Stray Losses', linewidth=2)
            self.ax_losses.set_xlabel('Time (s)')
            self.ax_losses.set_ylabel('Losses (kW)')
            self.ax_losses.set_title('Loss Breakdown')
            self.ax_losses.legend()
            self.ax_losses.grid(True, alpha=0.3)

            self.ax_torque.clear()
            self.ax_torque.plot(self.simulator.time_data, self.simulator.torque_data, 'm-', linewidth=2)
            self.ax_torque.set_xlabel('Time (s)')
            self.ax_torque.set_ylabel('Torque (Nm)')
            self.ax_torque.set_title('Shaft Torque')
            self.ax_torque.grid(True, alpha=0.3)

            self.ax_current.clear()
            self.ax_current.plot(self.simulator.time_data, current_arr, 'c-', linewidth=2)
            self.ax_current.set_xlabel('Time (s)')
            self.ax_current.set_ylabel('Current RMS (A)')
            self.ax_current.set_title('Motor Current')
            self.ax_current.grid(True, alpha=0.3)

            self.ax_efficiency.clear()
            self.ax_efficiency.plot(self.simulator.time_data, self.simulator.efficiency_data, 'g-', linewidth=2)
            self.ax_efficiency.set_xlabel('Time (s)')
            self.ax_efficiency.set_ylabel('Efficiency (%)')
            self.ax_efficiency.set_title('System Efficiency')
            self.ax_efficiency.grid(True, alpha=0.3)

            self.mp_canvas.draw()

    def simulation_complete(self):
        """Handle simulation completion"""
        self.stop_simulation()

        # Generate summary report
        if len(self.simulator.time_data) > 0:
            final_energy = self.simulator.energy_data[-1]
            avg_power = np.mean(self.simulator.power_data)
            max_temp = max(self.simulator.temp_data)
            max_torque = max(self.simulator.torque_data)
            avg_efficiency = np.mean(self.simulator.efficiency_data)

            summary = "="*60 + "\n"
            summary += "SIMULATION COMPLETE - SUMMARY REPORT\n"
            summary += "="*60 + "\n\n"
            summary += f"Simulation Time: {self.simulator.time_data[-1]:.2f} seconds\n"
            summary += f"Final Energy: {final_energy:.4f} kWh\n"
            summary += f"Average Power: {avg_power:.2f} kW\n"
            summary += f"Maximum Temperature: {max_temp:.2f} °C\n"
            summary += f"Maximum Torque: {max_torque:.2f} Nm\n"
            summary += f"Average Efficiency: {avg_efficiency:.2f} %\n"
            summary += "="*60 + "\n"

            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(1.0, summary)

        messagebox.showinfo("Complete", "Simulation completed successfully!")

    def calculate_economics(self):
        """Calculate and display economic analysis"""
        self.update_simulator_params()

        if len(self.simulator.energy_data) == 0:
            messagebox.showwarning("Warning", "Please run simulation first!")
            return

        # Get final values
        final_energy = self.simulator.energy_data[-1]
        distance_km = self.simulator.distance
        time_hours = self.simulator.time_data[-1] / 3600 if len(self.simulator.time_data) > 0 else 1

        econ_results = self.simulator.calculate_economic_metrics(final_energy, distance_km, time_hours)

        # Display results
        output = "="*70 + "\n"
        output += "ECONOMIC ANALYSIS REPORT\n"
        output += "="*70 + "\n\n"
        output += f"Report Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n"

        output += "INPUT PARAMETERS:\n"
        output += "-" * 70 + "\n"
        output += f"  Distance: {distance_km:.2f} km\n"
        output += f"  Energy Consumed/Returned: {final_energy:.4f} kWh\n"
        output += f"  Trip Duration: {time_hours:.2f} hours\n"
        output += f"  Electricity Cost: ${self.simulator.electricity_cost:.4f}/kWh\n"
        output += f"  Maintenance Cost: ${self.simulator.maintenance_cost:.4f}/km\n"
        output += f"  Depreciation Rate: {self.simulator.depreciation_rate*100:.2f}% per year\n\n"

        output += "COST BREAKDOWN:\n"
        output += "-" * 70 + "\n"
        output += f"  Energy Cost: ${econ_results['energy_cost']:.2f}\n"
        output += f"  Maintenance Cost: ${econ_results['maintenance_cost']:.2f}\n"
        output += f"  Total Operating Cost: ${econ_results['total_cost']:.2f}\n"
        output += f"  Cost per km: ${econ_results['cost_per_km']:.4f}/km\n\n"

        if econ_results['savings'] > 0:
            output += "REGENERATIVE BRAKING BENEFITS:\n"
            output += "-" * 70 + "\n"
            output += f"  Energy Returned to Grid: {abs(final_energy):.4f} kWh\n"
            output += f"  Energy Savings: ${econ_results['savings']:.2f}\n"
            output += f"  Net Cost: ${econ_results['total_cost'] - econ_results['savings']:.2f}\n\n"

        output += "EFFICIENCY METRICS:\n"
        output += "-" * 70 + "\n"
        output += f"  Energy Efficiency: {econ_results['energy_efficiency']:.6f} kWh/(km·tonne)\n"

        if len(self.simulator.efficiency_data) > 0:
            output += f"  Average System Efficiency: {np.mean(self.simulator.efficiency_data):.2f}%\n"
            output += f"  Peak Efficiency: {max(self.simulator.efficiency_data):.2f}%\n"

        output += "\n" + "="*70 + "\n"
        output += "ENVIRONMENTAL IMPACT:\n"
        output += "-" * 70 + "\n"

        # CO2 savings (approximate)
        co2_factor = 0.5  # kg CO2 per kWh
        co2_impact = abs(final_energy) * co2_factor

        if final_energy < 0:
            output += f"  CO2 Saved: {co2_impact:.2f} kg\n"
            output += f"  Equivalent Trees Planted: {co2_impact/22:.1f} trees\n"
        else:
            output += f"  CO2 Emissions: {co2_impact:.2f} kg\n"

        output += "="*70 + "\n"

        self.econ_text.delete(1.0, tk.END)
        self.econ_text.insert(1.0, output)

    def on_resize(self, event):
        """Handle window resize event"""
        # Auto-scale is handled by grid weight configuration
        pass

    def save_results(self):
        """Save results to file"""
        try:
            filename = f"train_sim_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            with open(filename, 'w') as f:
                f.write(self.results_text.get(1.0, tk.END))
            messagebox.showinfo("Success", f"Results saved to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save results: {str(e)}")

    def load_parameters(self):
        """Load parameters (placeholder)"""
        messagebox.showinfo("Info", "Load parameters feature - to be implemented")

    def export_results(self):
        """Export results to file"""
        self.save_results()

    def show_about(self):
        """Show about dialog"""
        about_text = """
        Advanced Train Energy & Multi-Physics Simulator
        Version 1.0

        Features:
        • Dynamic train energy calculation
        • Multi-physics simulation (electromagnetic-thermal-mechanical)
        • ODE solvers (RK45, Euler)
        • Real-time visualization
        • Economic analysis
        • Loss breakdown analysis
        • Thermal derating

        Developed for advanced electrical engineering applications
        """
        messagebox.showinfo("About", about_text)

    def show_documentation(self):
        """Show documentation"""
        doc_text = """
        DOCUMENTATION

        1. INPUT PARAMETERS TAB:
           - Adjust train and motor parameters using sliders
           - Click "Update Parameters" to apply changes

        2. DYNAMIC SIMULATION TAB:
           - Shows real-time plots of speed, power, energy, and temperature
           - Use Start/Pause/Stop/Reset buttons to control simulation

        3. MULTI-PHYSICS TAB:
           - Displays electromagnetic, thermal, and mechanical analysis
           - Shows loss breakdown and efficiency

        4. ECONOMIC ANALYSIS TAB:
           - Calculate operating costs and savings
           - Adjust economic parameters

        5. RESULTS SUMMARY TAB:
           - View detailed results and export to file

        ODE SOLVERS:
        - RK45: More accurate, adaptive step size
        - Euler: Faster, fixed step size

        For more information, consult electrical engineering references.
        """
        messagebox.showinfo("Documentation", doc_text)


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = TrainSimulatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
