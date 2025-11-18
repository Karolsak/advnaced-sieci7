#!/usr/bin/env python3
"""
Advanced 3-Phase Alternator Multi-Physics Simulation
Features:
- Voltage calculation for different power factors
- Real-time ODE solvers (RK45, Euler)
- Multi-physics simulation (electromagnetic-thermal-mechanical)
- Dynamic visualization and graphs
- Economic analysis and loss breakdown
- Advanced controls and thermal derating
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import math
from dataclasses import dataclass
from typing import Tuple, List
import threading
import time


@dataclass
class AlternatorParameters:
    """Alternator machine parameters"""
    kva_rating: float = 1000.0  # kVA
    voltage_rating: float = 3300.0  # V (line-to-line)
    phases: int = 3
    resistance: float = 0.5  # Ohm per phase
    sync_reactance: float = 5.0  # Ohm per phase
    frequency: float = 50.0  # Hz
    poles: int = 4
    inertia: float = 50.0  # kg.m^2
    ambient_temp: float = 25.0  # °C
    thermal_resistance: float = 0.5  # °C/W per phase
    thermal_capacitance: float = 5000.0  # J/°C per phase
    core_loss_constant: float = 0.02  # Core loss factor
    friction_coefficient: float = 0.001  # Mechanical friction
    stray_load_loss_factor: float = 0.01  # Stray load loss


class AlternatorCalculator:
    """Core calculation engine for alternator analysis"""

    def __init__(self, params: AlternatorParameters):
        self.params = params
        self.phase_voltage = params.voltage_rating / np.sqrt(3)
        self.full_load_current = (params.kva_rating * 1000) / (np.sqrt(3) * params.voltage_rating)

    def calculate_voltage_regulation(self, pf: float, pf_type: str = 'lagging',
                                    current: float = None, voltage: float = None) -> dict:
        """
        Calculate alternator voltage and regulation

        Args:
            pf: Power factor
            pf_type: 'lagging' or 'leading'
            current: Load current (A)
            voltage: Terminal voltage (V per phase)

        Returns:
            Dictionary with results
        """
        if current is None:
            current = self.full_load_current

        if voltage is None:
            voltage = self.phase_voltage

        # Calculate power factor angle
        phi = np.arccos(pf)
        if pf_type == 'leading':
            phi = -phi

        # Calculate impedance
        Z = np.sqrt(self.params.resistance**2 + self.params.sync_reactance**2)

        # Calculate voltage drops
        I_R = current * self.params.resistance
        I_Xs = current * self.params.sync_reactance

        # Calculate induced EMF (E0) using phasor diagram
        # E0 = sqrt((V*cos(phi) + I*R)^2 + (V*sin(phi) + I*Xs)^2)
        E0_real = voltage * np.cos(phi) + I_R
        E0_imag = voltage * np.sin(phi) + I_Xs
        E0 = np.sqrt(E0_real**2 + E0_imag**2)

        # Voltage regulation
        regulation = ((E0 - voltage) / voltage) * 100

        return {
            'terminal_voltage_phase': voltage,
            'terminal_voltage_line': voltage * np.sqrt(3),
            'induced_emf': E0,
            'load_current': current,
            'voltage_regulation': regulation,
            'power_factor': pf,
            'pf_type': pf_type,
            'impedance': Z,
            'voltage_drop_R': I_R,
            'voltage_drop_Xs': I_Xs
        }

    def calculate_terminal_voltage_same_excitation(self, original_pf: float,
                                                    original_pf_type: str,
                                                    new_pf: float,
                                                    new_pf_type: str) -> dict:
        """
        Calculate terminal voltage for same excitation and load current
        but different power factor
        """
        # First, calculate E0 from original conditions
        original_result = self.calculate_voltage_regulation(original_pf, original_pf_type)
        E0 = original_result['induced_emf']
        current = original_result['load_current']

        # Now solve for new terminal voltage with same E0 and current
        # E0^2 = (V*cos(phi) + I*R)^2 + (V*sin(phi) + I*Xs)^2
        # This is a quadratic equation in V

        phi = np.arccos(new_pf)
        if new_pf_type == 'leading':
            phi = -phi

        # Expanding: E0^2 = V^2 + 2*V*(I*R*cos(phi) + I*Xs*sin(phi)) + I^2*(R^2 + Xs^2)
        a = 1
        b = 2 * (current * self.params.resistance * np.cos(phi) +
                 current * self.params.sync_reactance * np.sin(phi))
        c = (current**2 * (self.params.resistance**2 + self.params.sync_reactance**2) - E0**2)

        # Solve quadratic equation
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return None

        V1 = (-b + np.sqrt(discriminant)) / (2*a)
        V2 = (-b - np.sqrt(discriminant)) / (2*a)

        # Choose positive, reasonable voltage
        new_voltage = max(V1, V2) if V1 > 0 and V2 > 0 else (V1 if V1 > 0 else V2)

        return self.calculate_voltage_regulation(new_pf, new_pf_type, current, new_voltage)

    def calculate_losses(self, current: float, voltage: float, pf: float,
                        speed_rpm: float = None) -> dict:
        """Calculate detailed loss breakdown"""
        if speed_rpm is None:
            speed_rpm = 120 * self.params.frequency / self.params.poles

        # Copper losses (I^2 * R for 3 phases)
        copper_loss = 3 * current**2 * self.params.resistance

        # Core/Iron losses (proportional to voltage^2)
        core_loss = 3 * self.params.core_loss_constant * voltage**2

        # Mechanical friction losses
        angular_velocity = speed_rpm * 2 * np.pi / 60
        friction_loss = self.params.friction_coefficient * angular_velocity**2

        # Stray load losses
        stray_loss = self.params.stray_load_loss_factor * (3 * voltage * current * pf)

        total_loss = copper_loss + core_loss + friction_loss + stray_loss

        # Calculate efficiency
        output_power = 3 * voltage * current * pf
        input_power = output_power + total_loss
        efficiency = (output_power / input_power * 100) if input_power > 0 else 0

        return {
            'copper_loss': copper_loss,
            'core_loss': core_loss,
            'friction_loss': friction_loss,
            'stray_loss': stray_loss,
            'total_loss': total_loss,
            'output_power': output_power,
            'input_power': input_power,
            'efficiency': efficiency
        }

    def thermal_model_ode(self, t: float, y: np.ndarray, current: float) -> np.ndarray:
        """
        ODE system for thermal dynamics
        dy/dt = f(t, y)
        y[0] = winding temperature
        y[1] = core temperature
        """
        T_winding = y[0]
        T_core = y[1]

        # Heat generation
        P_copper = 3 * current**2 * self.params.resistance
        P_core = 3 * self.params.core_loss_constant * self.phase_voltage**2

        # Heat transfer
        dT_winding_dt = (P_copper - (T_winding - self.params.ambient_temp) /
                         self.params.thermal_resistance) / self.params.thermal_capacitance

        dT_core_dt = (P_core - (T_core - self.params.ambient_temp) /
                     self.params.thermal_resistance) / self.params.thermal_capacitance

        return np.array([dT_winding_dt, dT_core_dt])

    def electromagnetic_mechanical_ode(self, t: float, y: np.ndarray,
                                      torque_load: float) -> np.ndarray:
        """
        ODE system for electromagnetic-mechanical dynamics
        y[0] = rotor angle (rad)
        y[1] = rotor speed (rad/s)
        y[2] = field current (A)
        """
        delta = y[0]
        omega = y[1]
        I_f = y[2]

        omega_sync = 2 * np.pi * self.params.frequency

        # Electromagnetic torque (simplified)
        T_em = 3 * self.phase_voltage * I_f * np.sin(delta) / omega_sync

        # Friction torque
        T_friction = self.params.friction_coefficient * omega

        # Equations of motion
        d_delta_dt = omega - omega_sync
        d_omega_dt = (T_em - torque_load - T_friction) / self.params.inertia
        d_If_dt = 0  # Field current constant (no field control in this simplified model)

        return np.array([d_delta_dt, d_omega_dt, d_If_dt])

    def solve_thermal_transient(self, current: float, time_span: Tuple[float, float],
                               initial_temps: List[float], method: str = 'RK45') -> dict:
        """Solve thermal transient using ODE solver"""
        if method.upper() == 'RK45':
            sol = solve_ivp(
                lambda t, y: self.thermal_model_ode(t, y, current),
                time_span,
                initial_temps,
                method='RK45',
                dense_output=True,
                max_step=1.0
            )
        else:  # Euler method
            dt = 0.1
            t_eval = np.arange(time_span[0], time_span[1], dt)
            y = np.zeros((len(initial_temps), len(t_eval)))
            y[:, 0] = initial_temps

            for i in range(1, len(t_eval)):
                dydt = self.thermal_model_ode(t_eval[i-1], y[:, i-1], current)
                y[:, i] = y[:, i-1] + dt * dydt

            sol = type('obj', (object,), {
                't': t_eval,
                'y': y,
                'success': True
            })()

        return sol

    def calculate_mechanical_stress(self, torque: float, speed_rpm: float) -> dict:
        """Calculate mechanical stress on shaft and bearings"""
        angular_velocity = speed_rpm * 2 * np.pi / 60

        # Shaft stress (assuming solid circular shaft, simplified)
        shaft_diameter = 0.1  # meters (assumed)
        polar_moment = np.pi * shaft_diameter**4 / 32
        max_shear_stress = torque * (shaft_diameter/2) / polar_moment

        # Bearing loads (simplified radial load)
        bearing_radial_load = abs(torque / (shaft_diameter/2))

        # Power transmission
        mechanical_power = torque * angular_velocity

        return {
            'torque': torque,
            'speed_rpm': speed_rpm,
            'angular_velocity': angular_velocity,
            'shaft_shear_stress': max_shear_stress / 1e6,  # MPa
            'bearing_radial_load': bearing_radial_load / 1000,  # kN
            'mechanical_power': mechanical_power / 1000  # kW
        }


class AdvancedAlternatorSimulatorGUI:
    """Main GUI application for alternator simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced 3-Phase Alternator Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Initialize parameters and calculator
        self.params = AlternatorParameters()
        self.calculator = AlternatorCalculator(self.params)

        # Simulation state
        self.simulation_running = False
        self.simulation_thread = None
        self.time_data = []
        self.temp_winding_data = []
        self.temp_core_data = []

        # Create GUI
        self.create_gui()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def create_gui(self):
        """Create the main GUI layout"""
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.tab_main = ttk.Frame(self.notebook)
        self.tab_dynamic = ttk.Frame(self.notebook)
        self.tab_losses = ttk.Frame(self.notebook)
        self.tab_thermal = ttk.Frame(self.notebook)
        self.tab_mechanical = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_main, text='Main Calculation')
        self.notebook.add(self.tab_dynamic, text='Dynamic Simulation')
        self.notebook.add(self.tab_losses, text='Loss Analysis')
        self.notebook.add(self.tab_thermal, text='Thermal Analysis')
        self.notebook.add(self.tab_mechanical, text='Mechanical Stress')
        self.notebook.add(self.tab_economic, text='Economic Analysis')

        # Build each tab
        self.build_main_tab()
        self.build_dynamic_tab()
        self.build_losses_tab()
        self.build_thermal_tab()
        self.build_mechanical_tab()
        self.build_economic_tab()

    def build_main_tab(self):
        """Build main calculation tab"""
        # Input frame
        input_frame = ttk.LabelFrame(self.tab_main, text="Alternator Parameters", padding=10)
        input_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')

        # Parameters
        params_info = [
            ("kVA Rating", "kva_rating", 100, 5000, "kVA"),
            ("Voltage Rating (L-L)", "voltage_rating", 100, 15000, "V"),
            ("Resistance per phase", "resistance", 0.1, 10, "Ω"),
            ("Sync Reactance per phase", "sync_reactance", 0.5, 50, "Ω"),
            ("Frequency", "frequency", 40, 60, "Hz"),
        ]

        self.param_sliders = {}
        for i, (label, attr, min_val, max_val, unit) in enumerate(params_info):
            ttk.Label(input_frame, text=f"{label}:").grid(row=i, column=0, sticky='w', pady=5)

            slider = ttk.Scale(input_frame, from_=min_val, to=max_val, orient='horizontal',
                             command=lambda v, a=attr: self.update_param(a, v))
            slider.set(getattr(self.params, attr))
            slider.grid(row=i, column=1, sticky='ew', padx=5, pady=5)

            value_label = ttk.Label(input_frame, text=f"{getattr(self.params, attr):.2f} {unit}")
            value_label.grid(row=i, column=2, sticky='w', pady=5)

            self.param_sliders[attr] = (slider, value_label, unit)

        # Operating conditions frame
        cond_frame = ttk.LabelFrame(self.tab_main, text="Operating Conditions", padding=10)
        cond_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nsew')

        ttk.Label(cond_frame, text="Original Power Factor:").grid(row=0, column=0, sticky='w', pady=5)
        self.orig_pf_var = tk.DoubleVar(value=0.80)
        ttk.Scale(cond_frame, from_=0.5, to=1.0, orient='horizontal',
                 variable=self.orig_pf_var).grid(row=0, column=1, sticky='ew', padx=5, pady=5)
        self.orig_pf_label = ttk.Label(cond_frame, text="0.80")
        self.orig_pf_label.grid(row=0, column=2, sticky='w', pady=5)
        self.orig_pf_var.trace('w', self.update_pf_labels)

        ttk.Label(cond_frame, text="Original PF Type:").grid(row=1, column=0, sticky='w', pady=5)
        self.orig_pf_type = tk.StringVar(value='lagging')
        ttk.Radiobutton(cond_frame, text="Lagging", variable=self.orig_pf_type,
                       value='lagging').grid(row=1, column=1, sticky='w', pady=5)
        ttk.Radiobutton(cond_frame, text="Leading", variable=self.orig_pf_type,
                       value='leading').grid(row=1, column=2, sticky='w', pady=5)

        ttk.Label(cond_frame, text="New Power Factor:").grid(row=2, column=0, sticky='w', pady=5)
        self.new_pf_var = tk.DoubleVar(value=0.80)
        ttk.Scale(cond_frame, from_=0.5, to=1.0, orient='horizontal',
                 variable=self.new_pf_var).grid(row=2, column=1, sticky='ew', padx=5, pady=5)
        self.new_pf_label = ttk.Label(cond_frame, text="0.80")
        self.new_pf_label.grid(row=2, column=2, sticky='w', pady=5)
        self.new_pf_var.trace('w', self.update_pf_labels)

        ttk.Label(cond_frame, text="New PF Type:").grid(row=3, column=0, sticky='w', pady=5)
        self.new_pf_type = tk.StringVar(value='leading')
        ttk.Radiobutton(cond_frame, text="Lagging", variable=self.new_pf_type,
                       value='lagging').grid(row=3, column=1, sticky='w', pady=5)
        ttk.Radiobutton(cond_frame, text="Leading", variable=self.new_pf_type,
                       value='leading').grid(row=3, column=2, sticky='w', pady=5)

        # Calculate button
        ttk.Button(cond_frame, text="Calculate", command=self.calculate_main,
                  style='Accent.TButton').grid(row=4, column=0, columnspan=3, pady=10)

        # Results frame
        results_frame = ttk.LabelFrame(self.tab_main, text="Calculation Results", padding=10)
        results_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky='nsew')

        self.results_text = tk.Text(results_frame, height=15, width=80, font=('Courier', 10))
        self.results_text.pack(fill='both', expand=True)

        scrollbar = ttk.Scrollbar(results_frame, command=self.results_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Configure grid weights
        self.tab_main.columnconfigure(0, weight=1)
        self.tab_main.columnconfigure(1, weight=1)
        self.tab_main.rowconfigure(1, weight=1)

    def build_dynamic_tab(self):
        """Build dynamic simulation tab"""
        # Control frame
        control_frame = ttk.LabelFrame(self.tab_dynamic, text="Simulation Controls", padding=10)
        control_frame.pack(side='top', fill='x', padx=10, pady=10)

        ttk.Label(control_frame, text="Load Current (A):").grid(row=0, column=0, pady=5)
        self.dyn_current_var = tk.DoubleVar(value=self.calculator.full_load_current)
        ttk.Scale(control_frame, from_=0, to=self.calculator.full_load_current*1.5,
                 orient='horizontal', variable=self.dyn_current_var,
                 length=200).grid(row=0, column=1, padx=5, pady=5)
        self.dyn_current_label = ttk.Label(control_frame,
                                          text=f"{self.calculator.full_load_current:.2f} A")
        self.dyn_current_label.grid(row=0, column=2, pady=5)
        self.dyn_current_var.trace('w', lambda *args: self.dyn_current_label.config(
            text=f"{self.dyn_current_var.get():.2f} A"))

        ttk.Label(control_frame, text="ODE Solver:").grid(row=0, column=3, padx=10, pady=5)
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(control_frame, text="RK45", variable=self.solver_var,
                       value='RK45').grid(row=0, column=4, pady=5)
        ttk.Radiobutton(control_frame, text="Euler", variable=self.solver_var,
                       value='Euler').grid(row=0, column=5, pady=5)

        # Buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=1, column=0, columnspan=6, pady=10)

        self.start_btn = ttk.Button(btn_frame, text="Start", command=self.start_simulation)
        self.start_btn.pack(side='left', padx=5)

        self.stop_btn = ttk.Button(btn_frame, text="Stop", command=self.stop_simulation,
                                   state='disabled')
        self.stop_btn.pack(side='left', padx=5)

        ttk.Button(btn_frame, text="Reset", command=self.reset_simulation).pack(side='left', padx=5)

        # Graph frame
        graph_frame = ttk.LabelFrame(self.tab_dynamic, text="Real-Time Thermal Response",
                                     padding=10)
        graph_frame.pack(side='top', fill='both', expand=True, padx=10, pady=10)

        self.fig_dynamic = Figure(figsize=(10, 6), dpi=100)
        self.ax_dynamic = self.fig_dynamic.add_subplot(111)
        self.ax_dynamic.set_xlabel('Time (s)')
        self.ax_dynamic.set_ylabel('Temperature (°C)')
        self.ax_dynamic.set_title('Thermal Transient Response')
        self.ax_dynamic.grid(True, alpha=0.3)

        self.canvas_dynamic = FigureCanvasTkAgg(self.fig_dynamic, graph_frame)
        self.canvas_dynamic.get_tk_widget().pack(fill='both', expand=True)

    def build_losses_tab(self):
        """Build loss analysis tab"""
        # Input frame
        input_frame = ttk.LabelFrame(self.tab_losses, text="Operating Point", padding=10)
        input_frame.pack(side='top', fill='x', padx=10, pady=10)

        ttk.Label(input_frame, text="Load (% of rated):").grid(row=0, column=0, pady=5)
        self.load_percent_var = tk.DoubleVar(value=100)
        ttk.Scale(input_frame, from_=0, to=150, orient='horizontal',
                 variable=self.load_percent_var, length=300).grid(row=0, column=1, padx=5, pady=5)
        self.load_label = ttk.Label(input_frame, text="100%")
        self.load_label.grid(row=0, column=2, pady=5)
        self.load_percent_var.trace('w', lambda *args: self.load_label.config(
            text=f"{self.load_percent_var.get():.1f}%"))

        ttk.Button(input_frame, text="Calculate Losses",
                  command=self.calculate_losses).grid(row=0, column=3, padx=10, pady=5)

        # Results frame
        results_frame = ttk.Frame(self.tab_losses)
        results_frame.pack(side='left', fill='both', expand=True, padx=10, pady=10)

        # Text results
        self.losses_text = tk.Text(results_frame, height=15, width=50, font=('Courier', 10))
        self.losses_text.pack(side='left', fill='both', expand=True)

        # Pie chart
        self.fig_losses = Figure(figsize=(6, 6), dpi=100)
        self.ax_losses = self.fig_losses.add_subplot(111)
        self.canvas_losses = FigureCanvasTkAgg(self.fig_losses, self.tab_losses)
        self.canvas_losses.get_tk_widget().pack(side='right', fill='both', expand=True,
                                                padx=10, pady=10)

    def build_thermal_tab(self):
        """Build thermal analysis tab"""
        # Parameters frame
        param_frame = ttk.LabelFrame(self.tab_thermal, text="Thermal Parameters", padding=10)
        param_frame.pack(side='top', fill='x', padx=10, pady=10)

        ttk.Label(param_frame, text="Ambient Temperature (°C):").grid(row=0, column=0, pady=5)
        self.ambient_temp_var = tk.DoubleVar(value=25)
        ttk.Scale(param_frame, from_=0, to=50, orient='horizontal',
                 variable=self.ambient_temp_var, length=200).grid(row=0, column=1, padx=5, pady=5)
        self.ambient_label = ttk.Label(param_frame, text="25°C")
        self.ambient_label.grid(row=0, column=2, pady=5)
        self.ambient_temp_var.trace('w', lambda *args: self.ambient_label.config(
            text=f"{self.ambient_temp_var.get():.1f}°C"))

        ttk.Label(param_frame, text="Thermal Derating:").grid(row=1, column=0, pady=5)
        self.derating_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(param_frame, variable=self.derating_var,
                       text="Enable automatic derating at high temperatures").grid(
                           row=1, column=1, columnspan=2, sticky='w', pady=5)

        ttk.Button(param_frame, text="Calculate Thermal Profile",
                  command=self.calculate_thermal).grid(row=2, column=0, columnspan=3, pady=10)

        # Graph frame
        graph_frame = ttk.LabelFrame(self.tab_thermal, text="Temperature Distribution", padding=10)
        graph_frame.pack(side='top', fill='both', expand=True, padx=10, pady=10)

        self.fig_thermal = Figure(figsize=(10, 6), dpi=100)
        self.ax_thermal = self.fig_thermal.add_subplot(111)
        self.canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, graph_frame)
        self.canvas_thermal.get_tk_widget().pack(fill='both', expand=True)

    def build_mechanical_tab(self):
        """Build mechanical stress analysis tab"""
        # Input frame
        input_frame = ttk.LabelFrame(self.tab_mechanical, text="Mechanical Parameters", padding=10)
        input_frame.pack(side='top', fill='x', padx=10, pady=10)

        ttk.Label(input_frame, text="Load Torque (N·m):").grid(row=0, column=0, pady=5)
        self.torque_var = tk.DoubleVar(value=1000)
        ttk.Scale(input_frame, from_=0, to=5000, orient='horizontal',
                 variable=self.torque_var, length=300).grid(row=0, column=1, padx=5, pady=5)
        self.torque_label = ttk.Label(input_frame, text="1000 N·m")
        self.torque_label.grid(row=0, column=2, pady=5)
        self.torque_var.trace('w', lambda *args: self.torque_label.config(
            text=f"{self.torque_var.get():.1f} N·m"))

        ttk.Label(input_frame, text="Speed (RPM):").grid(row=1, column=0, pady=5)
        self.speed_var = tk.DoubleVar(value=1500)
        ttk.Scale(input_frame, from_=0, to=3000, orient='horizontal',
                 variable=self.speed_var, length=300).grid(row=1, column=1, padx=5, pady=5)
        self.speed_label = ttk.Label(input_frame, text="1500 RPM")
        self.speed_label.grid(row=1, column=2, pady=5)
        self.speed_var.trace('w', lambda *args: self.speed_label.config(
            text=f"{self.speed_var.get():.1f} RPM"))

        ttk.Button(input_frame, text="Analyze Mechanical Stress",
                  command=self.calculate_mechanical).grid(row=2, column=0, columnspan=3, pady=10)

        # Results frame
        self.mech_text = tk.Text(self.tab_mechanical, height=20, font=('Courier', 10))
        self.mech_text.pack(side='top', fill='both', expand=True, padx=10, pady=10)

    def build_economic_tab(self):
        """Build economic analysis tab"""
        # Parameters frame
        param_frame = ttk.LabelFrame(self.tab_economic, text="Economic Parameters", padding=10)
        param_frame.pack(side='top', fill='x', padx=10, pady=10)

        ttk.Label(param_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, pady=5)
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(param_frame, textvariable=self.elec_cost_var, width=10).grid(
            row=0, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="Operating Hours/Year:").grid(row=1, column=0, pady=5)
        self.op_hours_var = tk.DoubleVar(value=8760)
        ttk.Entry(param_frame, textvariable=self.op_hours_var, width=10).grid(
            row=1, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="Maintenance Cost/Year ($):").grid(row=2, column=0, pady=5)
        self.maint_cost_var = tk.DoubleVar(value=5000)
        ttk.Entry(param_frame, textvariable=self.maint_cost_var, width=10).grid(
            row=2, column=1, padx=5, pady=5)

        ttk.Button(param_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=3, column=0, columnspan=2, pady=10)

        # Results frame
        self.econ_text = tk.Text(self.tab_economic, height=20, font=('Courier', 10))
        self.econ_text.pack(side='top', fill='both', expand=True, padx=10, pady=10)

    def update_param(self, attr, value):
        """Update parameter value"""
        value = float(value)
        setattr(self.params, attr, value)
        self.calculator = AlternatorCalculator(self.params)

        # Update label
        if attr in self.param_sliders:
            _, label, unit = self.param_sliders[attr]
            label.config(text=f"{value:.2f} {unit}")

    def update_pf_labels(self, *args):
        """Update power factor labels"""
        self.orig_pf_label.config(text=f"{self.orig_pf_var.get():.2f}")
        self.new_pf_label.config(text=f"{self.new_pf_var.get():.2f}")

    def calculate_main(self):
        """Perform main voltage calculation"""
        try:
            # Get parameters
            orig_pf = self.orig_pf_var.get()
            orig_type = self.orig_pf_type.get()
            new_pf = self.new_pf_var.get()
            new_type = self.new_pf_type.get()

            # Calculate original condition
            orig_result = self.calculator.calculate_voltage_regulation(orig_pf, orig_type)

            # Calculate new condition with same excitation
            new_result = self.calculator.calculate_terminal_voltage_same_excitation(
                orig_pf, orig_type, new_pf, new_type)

            if new_result is None:
                messagebox.showerror("Error", "Cannot calculate voltage for given conditions")
                return

            # Display results
            self.results_text.delete(1.0, tk.END)

            output = "="*70 + "\n"
            output += "ALTERNATOR VOLTAGE CALCULATION RESULTS\n"
            output += "="*70 + "\n\n"

            output += "ORIGINAL CONDITIONS:\n"
            output += f"  Power Factor: {orig_result['power_factor']:.3f} {orig_result['pf_type']}\n"
            output += f"  Terminal Voltage (Phase): {orig_result['terminal_voltage_phase']:.2f} V\n"
            output += f"  Terminal Voltage (Line): {orig_result['terminal_voltage_line']:.2f} V\n"
            output += f"  Load Current: {orig_result['load_current']:.2f} A\n"
            output += f"  Induced EMF: {orig_result['induced_emf']:.2f} V\n"
            output += f"  Voltage Regulation: {orig_result['voltage_regulation']:.2f} %\n"
            output += f"  Voltage Drop (R): {orig_result['voltage_drop_R']:.2f} V\n"
            output += f"  Voltage Drop (Xs): {orig_result['voltage_drop_Xs']:.2f} V\n\n"

            output += "NEW CONDITIONS (Same Excitation & Current):\n"
            output += f"  Power Factor: {new_result['power_factor']:.3f} {new_result['pf_type']}\n"
            output += f"  Terminal Voltage (Phase): {new_result['terminal_voltage_phase']:.2f} V\n"
            output += f"  Terminal Voltage (Line): {new_result['terminal_voltage_line']:.2f} V\n"
            output += f"  Load Current: {new_result['load_current']:.2f} A\n"
            output += f"  Induced EMF: {new_result['induced_emf']:.2f} V\n"
            output += f"  Voltage Regulation: {new_result['voltage_regulation']:.2f} %\n"
            output += f"  Voltage Drop (R): {new_result['voltage_drop_R']:.2f} V\n"
            output += f"  Voltage Drop (Xs): {new_result['voltage_drop_Xs']:.2f} V\n\n"

            output += "="*70 + "\n"
            output += f"VOLTAGE CHANGE: {new_result['terminal_voltage_line'] - orig_result['terminal_voltage_line']:.2f} V\n"
            output += f"PERCENTAGE CHANGE: {((new_result['terminal_voltage_line'] - orig_result['terminal_voltage_line']) / orig_result['terminal_voltage_line'] * 100):.2f} %\n"
            output += "="*70 + "\n"

            self.results_text.insert(1.0, output)

        except Exception as e:
            messagebox.showerror("Error", f"Calculation error: {str(e)}")

    def start_simulation(self):
        """Start dynamic simulation"""
        if not self.simulation_running:
            self.simulation_running = True
            self.start_btn.config(state='disabled')
            self.stop_btn.config(state='normal')

            # Reset data
            self.time_data = []
            self.temp_winding_data = []
            self.temp_core_data = []

            # Start simulation thread
            self.simulation_thread = threading.Thread(target=self.run_simulation)
            self.simulation_thread.daemon = True
            self.simulation_thread.start()

    def stop_simulation(self):
        """Stop dynamic simulation"""
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        self.time_data = []
        self.temp_winding_data = []
        self.temp_core_data = []
        self.ax_dynamic.clear()
        self.ax_dynamic.set_xlabel('Time (s)')
        self.ax_dynamic.set_ylabel('Temperature (°C)')
        self.ax_dynamic.set_title('Thermal Transient Response')
        self.ax_dynamic.grid(True, alpha=0.3)
        self.canvas_dynamic.draw()

    def run_simulation(self):
        """Run the dynamic simulation in background thread"""
        current = self.dyn_current_var.get()
        method = self.solver_var.get()

        # Initial conditions
        initial_temps = [self.params.ambient_temp, self.params.ambient_temp]
        t_current = 0
        dt = 0.5  # Time step for updates

        while self.simulation_running and t_current < 300:  # 5 minutes max
            # Solve for next time step
            if method == 'RK45':
                sol = self.calculator.solve_thermal_transient(
                    current, (t_current, t_current + dt), initial_temps, 'RK45')
                if sol.success:
                    initial_temps = [sol.y[0][-1], sol.y[1][-1]]
            else:  # Euler
                dydt = self.calculator.thermal_model_ode(t_current, initial_temps, current)
                initial_temps = [initial_temps[0] + dt * dydt[0],
                               initial_temps[1] + dt * dydt[1]]

            t_current += dt

            # Store data
            self.time_data.append(t_current)
            self.temp_winding_data.append(initial_temps[0])
            self.temp_core_data.append(initial_temps[1])

            # Update plot
            self.root.after(0, self.update_dynamic_plot)

            time.sleep(0.1)  # Small delay for visualization

    def update_dynamic_plot(self):
        """Update dynamic plot"""
        self.ax_dynamic.clear()
        self.ax_dynamic.plot(self.time_data, self.temp_winding_data, 'r-',
                           label='Winding Temperature', linewidth=2)
        self.ax_dynamic.plot(self.time_data, self.temp_core_data, 'b-',
                           label='Core Temperature', linewidth=2)
        self.ax_dynamic.axhline(y=self.params.ambient_temp, color='g', linestyle='--',
                              label='Ambient', alpha=0.7)
        self.ax_dynamic.set_xlabel('Time (s)')
        self.ax_dynamic.set_ylabel('Temperature (°C)')
        self.ax_dynamic.set_title('Thermal Transient Response')
        self.ax_dynamic.legend()
        self.ax_dynamic.grid(True, alpha=0.3)
        self.canvas_dynamic.draw()

    def calculate_losses(self):
        """Calculate and display loss breakdown"""
        try:
            load_percent = self.load_percent_var.get()
            current = self.calculator.full_load_current * load_percent / 100
            voltage = self.calculator.phase_voltage
            pf = 0.85  # Assumed power factor

            losses = self.calculator.calculate_losses(current, voltage, pf)

            # Display text results
            self.losses_text.delete(1.0, tk.END)
            output = "="*50 + "\n"
            output += "LOSS BREAKDOWN ANALYSIS\n"
            output += "="*50 + "\n\n"
            output += f"Operating Point: {load_percent:.1f}% of rated load\n"
            output += f"Load Current: {current:.2f} A\n\n"
            output += f"Copper Losses (I²R): {losses['copper_loss']:.2f} W\n"
            output += f"Core/Iron Losses: {losses['core_loss']:.2f} W\n"
            output += f"Mechanical Friction: {losses['friction_loss']:.2f} W\n"
            output += f"Stray Load Losses: {losses['stray_loss']:.2f} W\n"
            output += f"{'-'*50}\n"
            output += f"Total Losses: {losses['total_loss']:.2f} W\n\n"
            output += f"Output Power: {losses['output_power']/1000:.2f} kW\n"
            output += f"Input Power: {losses['input_power']/1000:.2f} kW\n"
            output += f"Efficiency: {losses['efficiency']:.2f} %\n"
            output += "="*50 + "\n"

            self.losses_text.insert(1.0, output)

            # Create pie chart
            self.ax_losses.clear()
            labels = ['Copper\nLosses', 'Core\nLosses', 'Friction\nLosses', 'Stray\nLosses']
            sizes = [losses['copper_loss'], losses['core_loss'],
                    losses['friction_loss'], losses['stray_loss']]
            colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']
            explode = (0.1, 0, 0, 0)

            self.ax_losses.pie(sizes, explode=explode, labels=labels, colors=colors,
                             autopct='%1.1f%%', shadow=True, startangle=90)
            self.ax_losses.set_title('Loss Distribution')
            self.canvas_losses.draw()

        except Exception as e:
            messagebox.showerror("Error", f"Loss calculation error: {str(e)}")

    def calculate_thermal(self):
        """Calculate thermal profile"""
        try:
            current = self.calculator.full_load_current
            self.params.ambient_temp = self.ambient_temp_var.get()

            # Solve thermal transient
            sol = self.calculator.solve_thermal_transient(
                current, (0, 600), [self.params.ambient_temp, self.params.ambient_temp], 'RK45')

            # Plot
            self.ax_thermal.clear()
            self.ax_thermal.plot(sol.t, sol.y[0], 'r-', label='Winding Temperature', linewidth=2)
            self.ax_thermal.plot(sol.t, sol.y[1], 'b-', label='Core Temperature', linewidth=2)
            self.ax_thermal.axhline(y=self.params.ambient_temp, color='g', linestyle='--',
                                   label='Ambient', alpha=0.7)

            # Add thermal limits
            self.ax_thermal.axhline(y=130, color='orange', linestyle='--',
                                   label='Warning (130°C)', alpha=0.7)
            self.ax_thermal.axhline(y=155, color='red', linestyle='--',
                                   label='Critical (155°C)', alpha=0.7)

            self.ax_thermal.set_xlabel('Time (s)')
            self.ax_thermal.set_ylabel('Temperature (°C)')
            self.ax_thermal.set_title('Thermal Profile - Steady State Analysis')
            self.ax_thermal.legend()
            self.ax_thermal.grid(True, alpha=0.3)
            self.canvas_thermal.draw()

            # Check derating
            if self.derating_var.get():
                max_temp = max(np.max(sol.y[0]), np.max(sol.y[1]))
                if max_temp > 130:
                    derating_factor = min(1.0, 130 / max_temp)
                    messagebox.showwarning("Thermal Derating",
                                         f"Temperature exceeds limits!\n"
                                         f"Maximum temperature: {max_temp:.1f}°C\n"
                                         f"Recommended derating factor: {derating_factor:.2f}\n"
                                         f"Derated capacity: {self.params.kva_rating * derating_factor:.1f} kVA")

        except Exception as e:
            messagebox.showerror("Error", f"Thermal calculation error: {str(e)}")

    def calculate_mechanical(self):
        """Calculate mechanical stress"""
        try:
            torque = self.torque_var.get()
            speed = self.speed_var.get()

            stress = self.calculator.calculate_mechanical_stress(torque, speed)

            # Display results
            self.mech_text.delete(1.0, tk.END)
            output = "="*60 + "\n"
            output += "MECHANICAL STRESS ANALYSIS\n"
            output += "="*60 + "\n\n"
            output += "INPUT PARAMETERS:\n"
            output += f"  Load Torque: {stress['torque']:.2f} N·m\n"
            output += f"  Rotational Speed: {stress['speed_rpm']:.2f} RPM\n"
            output += f"  Angular Velocity: {stress['angular_velocity']:.2f} rad/s\n\n"
            output += "SHAFT ANALYSIS:\n"
            output += f"  Maximum Shear Stress: {stress['shaft_shear_stress']:.2f} MPa\n"

            # Check against typical limits
            allowable_shear = 40  # MPa for typical steel shaft
            safety_factor = allowable_shear / stress['shaft_shear_stress'] if stress['shaft_shear_stress'] > 0 else float('inf')
            output += f"  Allowable Shear Stress: {allowable_shear} MPa\n"
            output += f"  Safety Factor: {safety_factor:.2f}\n"

            if safety_factor < 2:
                output += f"  WARNING: Low safety factor!\n"
            output += f"\n"

            output += "BEARING ANALYSIS:\n"
            output += f"  Radial Load: {stress['bearing_radial_load']:.2f} kN\n\n"

            output += "POWER TRANSMISSION:\n"
            output += f"  Mechanical Power: {stress['mechanical_power']:.2f} kW\n"
            output += "="*60 + "\n"

            # Add recommendations
            output += "\nRECOMMENDATIONS:\n"
            if safety_factor < 2:
                output += "  - Consider increasing shaft diameter\n"
                output += "  - Reduce operating torque or speed\n"
            else:
                output += "  - Shaft design is adequate\n"

            if stress['bearing_radial_load'] > 50:
                output += "  - High bearing loads detected\n"
                output += "  - Verify bearing specifications and lubrication\n"

            output += "="*60 + "\n"

            self.mech_text.insert(1.0, output)

        except Exception as e:
            messagebox.showerror("Error", f"Mechanical calculation error: {str(e)}")

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            elec_cost = self.elec_cost_var.get()
            op_hours = self.op_hours_var.get()
            maint_cost = self.maint_cost_var.get()

            # Calculate at different load levels
            load_levels = [25, 50, 75, 100]

            self.econ_text.delete(1.0, tk.END)
            output = "="*70 + "\n"
            output += "ECONOMIC ANALYSIS\n"
            output += "="*70 + "\n\n"
            output += f"Electricity Cost: ${elec_cost:.3f}/kWh\n"
            output += f"Operating Hours: {op_hours:.0f} hours/year\n"
            output += f"Maintenance Cost: ${maint_cost:.2f}/year\n\n"
            output += "="*70 + "\n\n"

            for load in load_levels:
                current = self.calculator.full_load_current * load / 100
                voltage = self.calculator.phase_voltage
                pf = 0.85

                losses = self.calculator.calculate_losses(current, voltage, pf)

                # Annual costs
                energy_loss_kwh = losses['total_loss'] / 1000 * op_hours
                energy_loss_cost = energy_loss_kwh * elec_cost
                total_annual_cost = energy_loss_cost + maint_cost

                output += f"LOAD LEVEL: {load}%\n"
                output += f"{'-'*70}\n"
                output += f"  Output Power: {losses['output_power']/1000:.2f} kW\n"
                output += f"  Total Losses: {losses['total_loss']/1000:.2f} kW\n"
                output += f"  Efficiency: {losses['efficiency']:.2f}%\n"
                output += f"  Annual Energy Loss: {energy_loss_kwh:.2f} kWh\n"
                output += f"  Annual Loss Cost: ${energy_loss_cost:.2f}\n"
                output += f"  Total Annual Cost: ${total_annual_cost:.2f}\n\n"

            # Life cycle cost (10 years)
            output += "="*70 + "\n"
            output += "10-YEAR LIFE CYCLE COST (at 100% load):\n"
            output += "="*70 + "\n"

            current = self.calculator.full_load_current
            losses = self.calculator.calculate_losses(current, self.calculator.phase_voltage, 0.85)
            annual_cost = (losses['total_loss'] / 1000 * op_hours * elec_cost) + maint_cost
            life_cycle_cost = annual_cost * 10

            output += f"  Annual Operating Cost: ${annual_cost:.2f}\n"
            output += f"  10-Year Life Cycle Cost: ${life_cycle_cost:.2f}\n"
            output += "="*70 + "\n"

            self.econ_text.insert(1.0, output)

        except Exception as e:
            messagebox.showerror("Error", f"Economic calculation error: {str(e)}")

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # This allows the GUI to auto-scale when window is resized
        # The pack and grid managers handle most of this automatically
        pass


def main():
    """Main entry point"""
    root = tk.Tk()
    app = AdvancedAlternatorSimulatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
