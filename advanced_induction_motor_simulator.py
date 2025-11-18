"""
Advanced Single-Phase Induction Motor Simulator
Multi-Physics Simulation with Electromagnetic, Thermal, and Mechanical Analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp, odeint
from dataclasses import dataclass
from typing import Tuple, List, Dict
import math


@dataclass
class MotorParameters:
    """Motor parameters dataclass"""
    rated_power: float = 185.0  # W
    poles: int = 4
    voltage: float = 110.0  # V
    frequency: float = 50.0  # Hz
    R1: float = 1.86  # Stator resistance (Ω)
    X1: float = 2.56  # Stator reactance (Ω)
    Xphi: float = 53.5  # Magnetizing reactance (Ω)
    R2: float = 3.56  # Rotor resistance (Ω)
    X2: float = 2.56  # Rotor reactance (Ω)
    core_loss: float = 3.5  # W
    friction_windage_loss: float = 13.5  # W
    slip: float = 0.05

    # Thermal parameters
    thermal_resistance: float = 5.0  # °C/W
    thermal_capacitance: float = 150.0  # J/°C
    ambient_temp: float = 25.0  # °C

    # Mechanical parameters
    inertia: float = 0.001  # kg·m²
    damping: float = 0.01  # N·m·s

    # Economic parameters
    electricity_cost: float = 0.12  # $/kWh
    maintenance_cost: float = 50.0  # $/year


class MotorCalculations:
    """Advanced motor calculation engine"""

    def __init__(self, params: MotorParameters):
        self.params = params
        self.omega_s = 2 * np.pi * params.frequency  # Synchronous angular frequency
        self.ns = 120 * params.frequency / params.poles  # Synchronous speed (rpm)

    def calculate_impedances(self, slip: float) -> Dict[str, complex]:
        """Calculate motor impedances"""
        p = self.params

        # Forward and backward impedances for single-phase motor
        Zf = complex(p.R2 / (2 * slip), p.X2 / 2)
        Zb = complex(p.R2 / (2 * (2 - slip)), p.X2 / 2)

        # Parallel combination with magnetizing reactance
        Xm = p.Xphi
        jXm = complex(0, Xm)

        Zf_parallel = (jXm * Zf) / (jXm + Zf)
        Zb_parallel = (jXm * Zb) / (jXm + Zb)

        # Total impedance
        Z1 = complex(p.R1, p.X1)
        Ztotal = Z1 + Zf_parallel + Zb_parallel

        return {
            'Zf': Zf,
            'Zb': Zb,
            'Zf_parallel': Zf_parallel,
            'Zb_parallel': Zb_parallel,
            'Z1': Z1,
            'Ztotal': Ztotal
        }

    def calculate_currents_and_power(self, slip: float) -> Dict[str, float]:
        """Calculate currents, power, and efficiency"""
        p = self.params

        # Get impedances
        Z = self.calculate_impedances(slip)

        # RMS voltage
        V = p.voltage

        # Stator current (RMS)
        I1 = V / abs(Z['Ztotal'])

        # Power factor
        pf = np.cos(np.angle(Z['Ztotal']))

        # Input power
        Pin = V * I1 * pf

        # Stator copper loss
        Pcu1 = I1**2 * p.R1

        # Forward and backward currents
        If = V / abs(Z['Z1'] + Z['Zf_parallel'])
        Ib = V / abs(Z['Z1'] + Z['Zb_parallel'])

        # Rotor copper losses
        Pcu2_forward = If**2 * (p.R2 / 2) * slip
        Pcu2_backward = Ib**2 * (p.R2 / 2) * (2 - slip)

        # Air gap power
        Pag_forward = If**2 * (p.R2 / (2 * slip))
        Pag_backward = Ib**2 * (p.R2 / (2 * (2 - slip)))

        # Developed power
        Pdev_forward = Pag_forward * (1 - slip)
        Pdev_backward = -Pag_backward * (slip - 1)  # Negative for backward

        Pdev_total = Pdev_forward + Pdev_backward

        # Output power
        Pout = Pdev_total - p.friction_windage_loss

        # Total losses
        total_losses = Pcu1 + Pcu2_forward + Pcu2_backward + p.core_loss + p.friction_windage_loss

        # Efficiency
        efficiency = (Pout / Pin) * 100 if Pin > 0 else 0

        # Torque
        omega_r = self.omega_s * (1 - slip)
        torque = Pdev_total / omega_r if omega_r > 0 else 0

        # Speed
        speed_rpm = self.ns * (1 - slip)

        return {
            'I1': I1,
            'If': If,
            'Ib': Ib,
            'Pin': Pin,
            'Pout': Pout,
            'Pcu1': Pcu1,
            'Pcu2_forward': Pcu2_forward,
            'Pcu2_backward': Pcu2_backward,
            'Pcore': p.core_loss,
            'Pfw': p.friction_windage_loss,
            'total_losses': total_losses,
            'efficiency': efficiency,
            'power_factor': pf,
            'torque': torque,
            'speed_rpm': speed_rpm,
            'slip': slip
        }

    def calculate_losses_breakdown(self, slip: float) -> Dict[str, float]:
        """Detailed loss breakdown"""
        results = self.calculate_currents_and_power(slip)

        total_loss = results['total_losses']

        return {
            'Copper Losses (Stator)': results['Pcu1'],
            'Copper Losses (Rotor Forward)': results['Pcu2_forward'],
            'Copper Losses (Rotor Backward)': results['Pcu2_backward'],
            'Core Losses': results['Pcore'],
            'Friction & Windage': results['Pfw'],
            'Total Losses': total_loss,
            'Loss Percentage': (total_loss / results['Pin'] * 100) if results['Pin'] > 0 else 0
        }


class ThermalModel:
    """Thermal analysis model"""

    def __init__(self, params: MotorParameters):
        self.params = params
        self.temperature = params.ambient_temp

    def thermal_ode(self, t: float, y: List[float], power_loss: float) -> List[float]:
        """Thermal differential equation"""
        T = y[0]
        p = self.params

        # dT/dt = (P_loss - (T - T_ambient) / R_th) / C_th
        dT_dt = (power_loss - (T - p.ambient_temp) / p.thermal_resistance) / p.thermal_capacitance

        return [dT_dt]

    def solve_thermal(self, power_loss: float, time_span: Tuple[float, float],
                     initial_temp: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """Solve thermal transient"""
        if initial_temp is None:
            initial_temp = self.params.ambient_temp

        t_eval = np.linspace(time_span[0], time_span[1], 100)

        sol = solve_ivp(
            lambda t, y: self.thermal_ode(t, y, power_loss),
            time_span,
            [initial_temp],
            t_eval=t_eval,
            method='RK45'
        )

        return sol.t, sol.y[0]

    def steady_state_temperature(self, power_loss: float) -> float:
        """Calculate steady-state temperature"""
        return self.params.ambient_temp + power_loss * self.params.thermal_resistance

    def derating_factor(self, temperature: float, max_temp: float = 155.0) -> float:
        """Calculate derating factor based on temperature"""
        if temperature >= max_temp:
            return 0.0
        elif temperature >= max_temp * 0.9:
            return 1.0 - (temperature - max_temp * 0.9) / (max_temp * 0.1)
        else:
            return 1.0


class MechanicalModel:
    """Mechanical dynamics model"""

    def __init__(self, params: MotorParameters):
        self.params = params
        self.calculator = MotorCalculations(params)

    def mechanical_ode(self, t: float, y: List[float], torque_load: float) -> List[float]:
        """Mechanical differential equation"""
        omega = y[0]  # Angular velocity
        p = self.params

        # Calculate slip from speed
        omega_s = 2 * np.pi * p.frequency
        slip = (omega_s - omega) / omega_s

        # Prevent invalid slip values
        slip = np.clip(slip, 0.001, 0.999)

        # Calculate motor torque
        results = self.calculator.calculate_currents_and_power(slip)
        torque_motor = results['torque']

        # d(omega)/dt = (T_motor - T_load - B*omega) / J
        domega_dt = (torque_motor - torque_load - p.damping * omega) / p.inertia

        return [domega_dt]

    def solve_mechanical_transient(self, torque_load: float, time_span: Tuple[float, float],
                                   initial_omega: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        """Solve mechanical transient using RK45"""
        t_eval = np.linspace(time_span[0], time_span[1], 200)

        sol = solve_ivp(
            lambda t, y: self.mechanical_ode(t, y, torque_load),
            time_span,
            [initial_omega],
            t_eval=t_eval,
            method='RK45',
            max_step=0.01
        )

        return sol.t, sol.y[0]

    def calculate_shaft_stress(self, torque: float, shaft_diameter: float = 0.01) -> Dict[str, float]:
        """Calculate shaft stress and bearing loads"""
        # Torsional stress: τ = 16*T / (π*d³)
        tau = 16 * torque / (np.pi * shaft_diameter**3)

        # Maximum shear stress
        tau_max = tau

        # Equivalent bearing load (simplified)
        bearing_load = abs(torque) / (shaft_diameter / 2)

        return {
            'torsional_stress': tau,
            'max_shear_stress': tau_max,
            'bearing_load': bearing_load
        }


class EconomicAnalysis:
    """Economic analysis module"""

    def __init__(self, params: MotorParameters):
        self.params = params

    def calculate_operating_costs(self, power_kw: float, hours_per_year: float = 8760) -> Dict[str, float]:
        """Calculate annual operating costs"""
        # Energy cost
        energy_cost_annual = power_kw * hours_per_year * self.params.electricity_cost

        # Total cost
        total_cost_annual = energy_cost_annual + self.params.maintenance_cost

        # Lifetime costs (assuming 15 years)
        lifetime_years = 15
        total_lifetime_cost = total_cost_annual * lifetime_years

        return {
            'energy_cost_annual': energy_cost_annual,
            'maintenance_cost_annual': self.params.maintenance_cost,
            'total_cost_annual': total_cost_annual,
            'total_lifetime_cost': total_lifetime_cost,
            'cost_per_hour': total_cost_annual / hours_per_year
        }

    def efficiency_comparison(self, efficiency: float, reference_efficiency: float = 85.0) -> Dict[str, float]:
        """Compare efficiency with reference"""
        power_kw = self.params.rated_power / 1000.0
        hours_per_year = 8760

        # Energy consumption
        energy_actual = (power_kw / (efficiency / 100)) * hours_per_year
        energy_reference = (power_kw / (reference_efficiency / 100)) * hours_per_year

        # Cost difference
        cost_actual = energy_actual * self.params.electricity_cost
        cost_reference = energy_reference * self.params.electricity_cost

        savings = cost_reference - cost_actual

        return {
            'energy_actual_kwh': energy_actual,
            'energy_reference_kwh': energy_reference,
            'cost_actual': cost_actual,
            'cost_reference': cost_reference,
            'annual_savings': savings,
            'savings_percentage': (savings / cost_reference * 100) if cost_reference > 0 else 0
        }


class AdvancedMotorSimulatorGUI:
    """Advanced Tkinter GUI for motor simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Single-Phase Induction Motor Simulator - Multi-Physics Analysis")
        self.root.geometry("1400x900")

        # Initialize parameters
        self.params = MotorParameters()
        self.calculator = MotorCalculations(self.params)
        self.thermal_model = ThermalModel(self.params)
        self.mechanical_model = MechanicalModel(self.params)
        self.economic_model = EconomicAnalysis(self.params)

        # Simulation state
        self.is_running = False
        self.simulation_time = 0.0
        self.time_history = []
        self.speed_history = []
        self.torque_history = []
        self.temp_history = []

        # Create GUI
        self.create_menu()
        self.create_main_interface()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

        # Initial calculation
        self.calculate_and_display()

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Reset Parameters", command=self.reset_parameters)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Simulation menu
        sim_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        sim_menu.add_command(label="Start", command=self.start_simulation)
        sim_menu.add_command(label="Stop", command=self.stop_simulation)
        sim_menu.add_command(label="Reset", command=self.reset_simulation)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def create_main_interface(self):
        """Create main interface with tabs"""
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_parameters_tab()
        self.create_steady_state_tab()
        self.create_dynamic_simulation_tab()
        self.create_thermal_analysis_tab()
        self.create_mechanical_analysis_tab()
        self.create_losses_tab()
        self.create_economic_analysis_tab()

        # Control panel at bottom
        self.create_control_panel()

    def create_parameters_tab(self):
        """Create parameters input tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Parameters")

        # Create canvas with scrollbar
        canvas = tk.Canvas(tab)
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Electrical Parameters
        elec_frame = ttk.LabelFrame(scrollable_frame, text="Electrical Parameters", padding=10)
        elec_frame.grid(row=0, column=0, padx=10, pady=5, sticky='ew')

        params_elec = [
            ('Rated Power (W):', 'rated_power', 0, 1000, 185),
            ('Voltage (V):', 'voltage', 0, 500, 110),
            ('Frequency (Hz):', 'frequency', 0, 100, 50),
            ('Number of Poles:', 'poles', 2, 12, 4),
            ('R1 - Stator Resistance (Ω):', 'R1', 0, 20, 1.86),
            ('X1 - Stator Reactance (Ω):', 'X1', 0, 20, 2.56),
            ('Xφ - Magnetizing Reactance (Ω):', 'Xphi', 0, 200, 53.5),
            ('R2 - Rotor Resistance (Ω):', 'R2', 0, 20, 3.56),
            ('X2 - Rotor Reactance (Ω):', 'X2', 0, 20, 2.56),
            ('Slip:', 'slip', 0, 1, 0.05),
        ]

        self.sliders = {}
        self.slider_labels = {}

        for i, (label, param, min_val, max_val, default) in enumerate(params_elec):
            ttk.Label(elec_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)

            slider = ttk.Scale(elec_frame, from_=min_val, to=max_val, orient='horizontal',
                             command=lambda v, p=param: self.update_parameter(p, v))
            slider.set(default)
            slider.grid(row=i, column=1, sticky='ew', padx=5, pady=2)

            value_label = ttk.Label(elec_frame, text=f"{default:.3f}")
            value_label.grid(row=i, column=2, sticky='w', pady=2)

            self.sliders[param] = slider
            self.slider_labels[param] = value_label

        elec_frame.columnconfigure(1, weight=1)

        # Loss Parameters
        loss_frame = ttk.LabelFrame(scrollable_frame, text="Loss Parameters", padding=10)
        loss_frame.grid(row=1, column=0, padx=10, pady=5, sticky='ew')

        params_loss = [
            ('Core Loss (W):', 'core_loss', 0, 50, 3.5),
            ('Friction & Windage Loss (W):', 'friction_windage_loss', 0, 50, 13.5),
        ]

        for i, (label, param, min_val, max_val, default) in enumerate(params_loss):
            ttk.Label(loss_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)

            slider = ttk.Scale(loss_frame, from_=min_val, to=max_val, orient='horizontal',
                             command=lambda v, p=param: self.update_parameter(p, v))
            slider.set(default)
            slider.grid(row=i, column=1, sticky='ew', padx=5, pady=2)

            value_label = ttk.Label(loss_frame, text=f"{default:.3f}")
            value_label.grid(row=i, column=2, sticky='w', pady=2)

            self.sliders[param] = slider
            self.slider_labels[param] = value_label

        loss_frame.columnconfigure(1, weight=1)

        # Thermal Parameters
        thermal_frame = ttk.LabelFrame(scrollable_frame, text="Thermal Parameters", padding=10)
        thermal_frame.grid(row=2, column=0, padx=10, pady=5, sticky='ew')

        params_thermal = [
            ('Thermal Resistance (°C/W):', 'thermal_resistance', 0, 20, 5.0),
            ('Thermal Capacitance (J/°C):', 'thermal_capacitance', 0, 500, 150.0),
            ('Ambient Temperature (°C):', 'ambient_temp', 0, 50, 25.0),
        ]

        for i, (label, param, min_val, max_val, default) in enumerate(params_thermal):
            ttk.Label(thermal_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)

            slider = ttk.Scale(thermal_frame, from_=min_val, to=max_val, orient='horizontal',
                             command=lambda v, p=param: self.update_parameter(p, v))
            slider.set(default)
            slider.grid(row=i, column=1, sticky='ew', padx=5, pady=2)

            value_label = ttk.Label(thermal_frame, text=f"{default:.3f}")
            value_label.grid(row=i, column=2, sticky='w', pady=2)

            self.sliders[param] = slider
            self.slider_labels[param] = value_label

        thermal_frame.columnconfigure(1, weight=1)

        # Mechanical Parameters
        mech_frame = ttk.LabelFrame(scrollable_frame, text="Mechanical Parameters", padding=10)
        mech_frame.grid(row=3, column=0, padx=10, pady=5, sticky='ew')

        params_mech = [
            ('Inertia (kg·m²):', 'inertia', 0, 0.01, 0.001),
            ('Damping (N·m·s):', 'damping', 0, 0.1, 0.01),
        ]

        for i, (label, param, min_val, max_val, default) in enumerate(params_mech):
            ttk.Label(mech_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)

            slider = ttk.Scale(mech_frame, from_=min_val, to=max_val, orient='horizontal',
                             command=lambda v, p=param: self.update_parameter(p, v))
            slider.set(default)
            slider.grid(row=i, column=1, sticky='ew', padx=5, pady=2)

            value_label = ttk.Label(mech_frame, text=f"{default:.4f}")
            value_label.grid(row=i, column=2, sticky='w', pady=2)

            self.sliders[param] = slider
            self.slider_labels[param] = value_label

        mech_frame.columnconfigure(1, weight=1)

        # Economic Parameters
        econ_frame = ttk.LabelFrame(scrollable_frame, text="Economic Parameters", padding=10)
        econ_frame.grid(row=4, column=0, padx=10, pady=5, sticky='ew')

        params_econ = [
            ('Electricity Cost ($/kWh):', 'electricity_cost', 0, 1, 0.12),
            ('Maintenance Cost ($/year):', 'maintenance_cost', 0, 500, 50.0),
        ]

        for i, (label, param, min_val, max_val, default) in enumerate(params_econ):
            ttk.Label(econ_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)

            slider = ttk.Scale(econ_frame, from_=min_val, to=max_val, orient='horizontal',
                             command=lambda v, p=param: self.update_parameter(p, v))
            slider.set(default)
            slider.grid(row=i, column=1, sticky='ew', padx=5, pady=2)

            value_label = ttk.Label(econ_frame, text=f"{default:.3f}")
            value_label.grid(row=i, column=2, sticky='w', pady=2)

            self.sliders[param] = slider
            self.slider_labels[param] = value_label

        econ_frame.columnconfigure(1, weight=1)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_steady_state_tab(self):
        """Create steady-state analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Steady-State Analysis")

        # Results frame
        results_frame = ttk.LabelFrame(tab, text="Calculation Results", padding=10)
        results_frame.pack(fill='both', expand=True, padx=10, pady=5)

        # Create text widget with scrollbar
        text_scroll = ttk.Scrollbar(results_frame)
        text_scroll.pack(side='right', fill='y')

        self.results_text = tk.Text(results_frame, height=20, width=60,
                                   yscrollcommand=text_scroll.set, font=('Courier', 10))
        self.results_text.pack(side='left', fill='both', expand=True)
        text_scroll.config(command=self.results_text.yview)

        # Graph frame
        graph_frame = ttk.LabelFrame(tab, text="Performance Curves", padding=10)
        graph_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.fig_steady = Figure(figsize=(12, 4), dpi=100)
        self.canvas_steady = FigureCanvasTkAgg(self.fig_steady, graph_frame)
        self.canvas_steady.get_tk_widget().pack(fill='both', expand=True)

    def create_dynamic_simulation_tab(self):
        """Create dynamic simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        # Controls
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(control_frame, text="ODE Solver:").grid(row=0, column=0, padx=5)
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_var,
                                   values=['RK45', 'Euler'], state='readonly', width=10)
        solver_combo.grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="Load Torque (N·m):").grid(row=0, column=2, padx=5)
        self.load_torque_var = tk.DoubleVar(value=0.5)
        ttk.Entry(control_frame, textvariable=self.load_torque_var, width=10).grid(row=0, column=3, padx=5)

        ttk.Label(control_frame, text="Simulation Time (s):").grid(row=0, column=4, padx=5)
        self.sim_time_var = tk.DoubleVar(value=2.0)
        ttk.Entry(control_frame, textvariable=self.sim_time_var, width=10).grid(row=0, column=5, padx=5)

        ttk.Button(control_frame, text="Run Transient",
                  command=self.run_transient_simulation).grid(row=0, column=6, padx=5)

        # Graphs
        graph_frame = ttk.Frame(tab)
        graph_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.fig_dynamic = Figure(figsize=(12, 6), dpi=100)
        self.canvas_dynamic = FigureCanvasTkAgg(self.fig_dynamic, graph_frame)
        self.canvas_dynamic.get_tk_widget().pack(fill='both', expand=True)

    def create_thermal_analysis_tab(self):
        """Create thermal analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Thermal Analysis")

        # Controls
        control_frame = ttk.LabelFrame(tab, text="Thermal Analysis Controls", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(control_frame, text="Max Temperature (°C):").grid(row=0, column=0, padx=5)
        self.max_temp_var = tk.DoubleVar(value=155.0)
        ttk.Entry(control_frame, textvariable=self.max_temp_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Button(control_frame, text="Run Thermal Analysis",
                  command=self.run_thermal_analysis).grid(row=0, column=2, padx=5)

        # Results
        results_frame = ttk.LabelFrame(tab, text="Thermal Results", padding=10)
        results_frame.pack(fill='x', padx=10, pady=5)

        self.thermal_results_text = tk.Text(results_frame, height=8, font=('Courier', 10))
        self.thermal_results_text.pack(fill='both', expand=True)

        # Graphs
        graph_frame = ttk.Frame(tab)
        graph_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.fig_thermal = Figure(figsize=(12, 5), dpi=100)
        self.canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, graph_frame)
        self.canvas_thermal.get_tk_widget().pack(fill='both', expand=True)

    def create_mechanical_analysis_tab(self):
        """Create mechanical analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Mechanical Analysis")

        # Controls
        control_frame = ttk.LabelFrame(tab, text="Mechanical Analysis Controls", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(control_frame, text="Shaft Diameter (m):").grid(row=0, column=0, padx=5)
        self.shaft_dia_var = tk.DoubleVar(value=0.01)
        ttk.Entry(control_frame, textvariable=self.shaft_dia_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Button(control_frame, text="Analyze Stress",
                  command=self.analyze_mechanical_stress).grid(row=0, column=2, padx=5)

        # Results
        results_frame = ttk.LabelFrame(tab, text="Mechanical Results", padding=10)
        results_frame.pack(fill='x', padx=10, pady=5)

        self.mechanical_results_text = tk.Text(results_frame, height=8, font=('Courier', 10))
        self.mechanical_results_text.pack(fill='both', expand=True)

        # Graphs
        graph_frame = ttk.Frame(tab)
        graph_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.fig_mechanical = Figure(figsize=(12, 5), dpi=100)
        self.canvas_mechanical = FigureCanvasTkAgg(self.fig_mechanical, graph_frame)
        self.canvas_mechanical.get_tk_widget().pack(fill='both', expand=True)

    def create_losses_tab(self):
        """Create losses breakdown tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Loss Analysis")

        # Graph frame
        graph_frame = ttk.Frame(tab)
        graph_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.fig_losses = Figure(figsize=(12, 6), dpi=100)
        self.canvas_losses = FigureCanvasTkAgg(self.fig_losses, graph_frame)
        self.canvas_losses.get_tk_widget().pack(fill='both', expand=True)

    def create_economic_analysis_tab(self):
        """Create economic analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Economic Analysis")

        # Controls
        control_frame = ttk.LabelFrame(tab, text="Operating Parameters", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(control_frame, text="Operating Hours/Year:").grid(row=0, column=0, padx=5)
        self.hours_per_year_var = tk.DoubleVar(value=8760)
        ttk.Entry(control_frame, textvariable=self.hours_per_year_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="Reference Efficiency (%):").grid(row=0, column=2, padx=5)
        self.ref_efficiency_var = tk.DoubleVar(value=85.0)
        ttk.Entry(control_frame, textvariable=self.ref_efficiency_var, width=10).grid(row=0, column=3, padx=5)

        ttk.Button(control_frame, text="Calculate Costs",
                  command=self.calculate_economics).grid(row=0, column=4, padx=5)

        # Results
        results_frame = ttk.LabelFrame(tab, text="Economic Results", padding=10)
        results_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.economic_results_text = tk.Text(results_frame, height=15, font=('Courier', 10))
        self.economic_results_text.pack(fill='both', expand=True)

    def create_control_panel(self):
        """Create control panel at bottom"""
        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill='x', padx=5, pady=5)

        ttk.Button(control_frame, text="START", command=self.start_simulation,
                  style='Success.TButton').pack(side='left', padx=5)
        ttk.Button(control_frame, text="STOP", command=self.stop_simulation,
                  style='Danger.TButton').pack(side='left', padx=5)
        ttk.Button(control_frame, text="RESET", command=self.reset_simulation).pack(side='left', padx=5)
        ttk.Button(control_frame, text="CALCULATE", command=self.calculate_and_display).pack(side='left', padx=5)

        self.status_label = ttk.Label(control_frame, text="Ready", relief='sunken')
        self.status_label.pack(side='right', padx=5, fill='x', expand=True)

    def update_parameter(self, param_name: str, value: str):
        """Update parameter value"""
        try:
            val = float(value)

            # Update parameter
            if param_name == 'poles':
                val = int(val)

            setattr(self.params, param_name, val)

            # Update label
            if param_name in ['inertia', 'damping']:
                self.slider_labels[param_name].config(text=f"{val:.4f}")
            else:
                self.slider_labels[param_name].config(text=f"{val:.3f}")

            # Recalculate
            self.calculator = MotorCalculations(self.params)
            self.thermal_model = ThermalModel(self.params)
            self.mechanical_model = MechanicalModel(self.params)
            self.economic_model = EconomicAnalysis(self.params)

            # Auto-update if not running
            if not self.is_running:
                self.calculate_and_display()

        except ValueError:
            pass

    def calculate_and_display(self):
        """Calculate and display results"""
        self.status_label.config(text="Calculating...")
        self.root.update()

        try:
            # Perform calculations
            results = self.calculator.calculate_currents_and_power(self.params.slip)

            # Display results
            self.display_steady_state_results(results)
            self.plot_performance_curves()
            self.plot_losses_breakdown()

            self.status_label.config(text=f"Calculation complete - Pout = {results['Pout']:.2f} W")

        except Exception as e:
            messagebox.showerror("Calculation Error", f"Error during calculation:\n{str(e)}")
            self.status_label.config(text="Error")

    def display_steady_state_results(self, results: Dict[str, float]):
        """Display steady-state results"""
        self.results_text.delete('1.0', tk.END)

        output = "="*70 + "\n"
        output += "  SINGLE-PHASE INDUCTION MOTOR ANALYSIS RESULTS\n"
        output += "="*70 + "\n\n"

        output += "MOTOR PARAMETERS:\n"
        output += "-"*70 + "\n"
        output += f"  Rated Power:        {self.params.rated_power:8.2f} W\n"
        output += f"  Voltage:            {self.params.voltage:8.2f} V (RMS)\n"
        output += f"  Frequency:          {self.params.frequency:8.2f} Hz\n"
        output += f"  Poles:              {self.params.poles:8d}\n"
        output += f"  Synchronous Speed:  {self.calculator.ns:8.2f} rpm\n"
        output += f"  Slip:               {self.params.slip:8.4f}\n\n"

        output += "ELECTRICAL CHARACTERISTICS:\n"
        output += "-"*70 + "\n"
        output += f"  Stator Current I1:  {results['I1']:8.3f} A (RMS)\n"
        output += f"  Forward Current:    {results['If']:8.3f} A (RMS)\n"
        output += f"  Backward Current:   {results['Ib']:8.3f} A (RMS)\n"
        output += f"  Power Factor:       {results['power_factor']:8.4f}\n\n"

        output += "POWER ANALYSIS:\n"
        output += "-"*70 + "\n"
        output += f"  Input Power:        {results['Pin']:8.2f} W\n"
        output += f"  Output Power:       {results['Pout']:8.2f} W  ← MECHANICAL POWER\n"
        output += f"  Efficiency:         {results['efficiency']:8.2f} %\n\n"

        output += "LOSSES BREAKDOWN:\n"
        output += "-"*70 + "\n"
        output += f"  Stator Cu Loss:     {results['Pcu1']:8.2f} W\n"
        output += f"  Rotor Cu (Fwd):     {results['Pcu2_forward']:8.2f} W\n"
        output += f"  Rotor Cu (Bwd):     {results['Pcu2_backward']:8.2f} W\n"
        output += f"  Core Loss:          {results['Pcore']:8.2f} W\n"
        output += f"  Friction/Windage:   {results['Pfw']:8.2f} W\n"
        output += f"  Total Losses:       {results['total_losses']:8.2f} W\n\n"

        output += "MECHANICAL PERFORMANCE:\n"
        output += "-"*70 + "\n"
        output += f"  Torque:             {results['torque']:8.4f} N·m\n"
        output += f"  Speed:              {results['speed_rpm']:8.2f} rpm\n\n"

        output += "="*70 + "\n"
        output += f"  ANSWER: Mechanical Power Output = {results['Pout']:.2f} W\n"
        output += "="*70 + "\n"

        self.results_text.insert('1.0', output)

    def plot_performance_curves(self):
        """Plot performance curves vs slip"""
        self.fig_steady.clear()

        # Generate slip range
        slip_range = np.linspace(0.01, 0.3, 50)

        torque = []
        power_out = []
        efficiency = []
        current = []

        for s in slip_range:
            res = self.calculator.calculate_currents_and_power(s)
            torque.append(res['torque'])
            power_out.append(res['Pout'])
            efficiency.append(res['efficiency'])
            current.append(res['I1'])

        # Create subplots
        ax1 = self.fig_steady.add_subplot(141)
        ax2 = self.fig_steady.add_subplot(142)
        ax3 = self.fig_steady.add_subplot(143)
        ax4 = self.fig_steady.add_subplot(144)

        # Plot torque
        ax1.plot(slip_range, torque, 'b-', linewidth=2)
        ax1.axvline(self.params.slip, color='r', linestyle='--', label=f'Operating Point')
        ax1.set_xlabel('Slip')
        ax1.set_ylabel('Torque (N·m)')
        ax1.set_title('Torque-Slip Curve')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Plot power output
        ax2.plot(slip_range, power_out, 'g-', linewidth=2)
        ax2.axvline(self.params.slip, color='r', linestyle='--')
        ax2.set_xlabel('Slip')
        ax2.set_ylabel('Power Output (W)')
        ax2.set_title('Power-Slip Curve')
        ax2.grid(True, alpha=0.3)

        # Plot efficiency
        ax3.plot(slip_range, efficiency, 'm-', linewidth=2)
        ax3.axvline(self.params.slip, color='r', linestyle='--')
        ax3.set_xlabel('Slip')
        ax3.set_ylabel('Efficiency (%)')
        ax3.set_title('Efficiency-Slip Curve')
        ax3.grid(True, alpha=0.3)

        # Plot current
        ax4.plot(slip_range, current, 'c-', linewidth=2)
        ax4.axvline(self.params.slip, color='r', linestyle='--')
        ax4.set_xlabel('Slip')
        ax4.set_ylabel('Current (A)')
        ax4.set_title('Current-Slip Curve')
        ax4.grid(True, alpha=0.3)

        self.fig_steady.tight_layout()
        self.canvas_steady.draw()

    def plot_losses_breakdown(self):
        """Plot losses breakdown"""
        self.fig_losses.clear()

        losses = self.calculator.calculate_losses_breakdown(self.params.slip)

        # Pie chart
        ax1 = self.fig_losses.add_subplot(121)

        loss_labels = ['Stator Cu', 'Rotor Cu (Fwd)', 'Rotor Cu (Bwd)', 'Core', 'Friction/Windage']
        loss_values = [
            losses['Copper Losses (Stator)'],
            losses['Copper Losses (Rotor Forward)'],
            losses['Copper Losses (Rotor Backward)'],
            losses['Core Losses'],
            losses['Friction & Windage']
        ]

        colors = ['#ff9999', '#ffcc99', '#ffff99', '#99ccff', '#cc99ff']
        ax1.pie(loss_values, labels=loss_labels, autopct='%1.1f%%', colors=colors, startangle=90)
        ax1.set_title('Loss Distribution')

        # Bar chart
        ax2 = self.fig_losses.add_subplot(122)
        y_pos = np.arange(len(loss_labels))
        ax2.barh(y_pos, loss_values, color=colors)
        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(loss_labels)
        ax2.set_xlabel('Loss (W)')
        ax2.set_title('Loss Breakdown')
        ax2.grid(True, alpha=0.3, axis='x')

        self.fig_losses.tight_layout()
        self.canvas_losses.draw()

    def run_transient_simulation(self):
        """Run transient simulation"""
        self.status_label.config(text="Running transient simulation...")
        self.root.update()

        try:
            load_torque = self.load_torque_var.get()
            sim_time = self.sim_time_var.get()

            # Run mechanical transient
            t, omega = self.mechanical_model.solve_mechanical_transient(
                load_torque, (0, sim_time), initial_omega=0.0
            )

            # Convert to RPM
            speed_rpm = omega * 60 / (2 * np.pi)

            # Calculate slip and torque
            slip_history = []
            torque_history = []
            power_history = []

            for w in omega:
                s = (self.mechanical_model.calculator.omega_s - w) / self.mechanical_model.calculator.omega_s
                s = np.clip(s, 0.001, 0.999)
                slip_history.append(s)

                res = self.calculator.calculate_currents_and_power(s)
                torque_history.append(res['torque'])
                power_history.append(res['Pout'])

            # Plot results
            self.fig_dynamic.clear()

            ax1 = self.fig_dynamic.add_subplot(221)
            ax2 = self.fig_dynamic.add_subplot(222)
            ax3 = self.fig_dynamic.add_subplot(223)
            ax4 = self.fig_dynamic.add_subplot(224)

            ax1.plot(t, speed_rpm, 'b-', linewidth=2)
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Speed (rpm)')
            ax1.set_title(f'Speed Response ({self.solver_var.get()} solver)')
            ax1.grid(True, alpha=0.3)

            ax2.plot(t, torque_history, 'g-', linewidth=2)
            ax2.axhline(load_torque, color='r', linestyle='--', label='Load Torque')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Torque (N·m)')
            ax2.set_title('Torque Response')
            ax2.grid(True, alpha=0.3)
            ax2.legend()

            ax3.plot(t, slip_history, 'm-', linewidth=2)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Slip')
            ax3.set_title('Slip Response')
            ax3.grid(True, alpha=0.3)

            ax4.plot(t, power_history, 'c-', linewidth=2)
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Power Output (W)')
            ax4.set_title('Power Response')
            ax4.grid(True, alpha=0.3)

            self.fig_dynamic.tight_layout()
            self.canvas_dynamic.draw()

            self.status_label.config(text=f"Transient simulation complete ({self.solver_var.get()})")

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation:\n{str(e)}")
            self.status_label.config(text="Error")

    def run_thermal_analysis(self):
        """Run thermal analysis"""
        self.status_label.config(text="Running thermal analysis...")
        self.root.update()

        try:
            # Get total losses at operating point
            results = self.calculator.calculate_currents_and_power(self.params.slip)
            power_loss = results['total_losses']

            # Solve thermal transient
            t_thermal, temp = self.thermal_model.solve_thermal(
                power_loss, (0, 3600), self.params.ambient_temp
            )

            # Calculate steady-state temperature
            T_ss = self.thermal_model.steady_state_temperature(power_loss)

            # Calculate derating factor
            max_temp = self.max_temp_var.get()
            derating = self.thermal_model.derating_factor(T_ss, max_temp)

            # Display results
            self.thermal_results_text.delete('1.0', tk.END)
            output = "THERMAL ANALYSIS RESULTS:\n"
            output += "="*70 + "\n"
            output += f"Total Losses:              {power_loss:8.2f} W\n"
            output += f"Steady-State Temperature:  {T_ss:8.2f} °C\n"
            output += f"Maximum Temperature:       {max_temp:8.2f} °C\n"
            output += f"Derating Factor:           {derating:8.2%}\n"
            output += f"Thermal Time Constant:     {self.params.thermal_resistance * self.params.thermal_capacitance:8.2f} s\n"

            if T_ss > max_temp:
                output += "\nWARNING: Motor exceeds maximum temperature!\n"
                output += "Recommendation: Reduce load or improve cooling\n"
            elif derating < 1.0:
                output += f"\nNOTE: Motor operating in derating zone ({derating:.1%} capacity)\n"
            else:
                output += "\nMotor operating within safe temperature limits.\n"

            self.thermal_results_text.insert('1.0', output)

            # Plot thermal response
            self.fig_thermal.clear()

            ax1 = self.fig_thermal.add_subplot(121)
            ax2 = self.fig_thermal.add_subplot(122)

            ax1.plot(t_thermal / 60, temp, 'r-', linewidth=2)
            ax1.axhline(T_ss, color='b', linestyle='--', label=f'Steady-State: {T_ss:.1f}°C')
            ax1.axhline(max_temp, color='k', linestyle='--', label=f'Max Temp: {max_temp:.1f}°C')
            ax1.set_xlabel('Time (minutes)')
            ax1.set_ylabel('Temperature (°C)')
            ax1.set_title('Thermal Transient Response')
            ax1.grid(True, alpha=0.3)
            ax1.legend()

            # Temperature vs load
            load_factors = np.linspace(0.5, 1.5, 20)
            temps_vs_load = []
            derating_vs_load = []

            for lf in load_factors:
                # Approximate loss scaling with load
                loss = power_loss * lf**2
                T = self.thermal_model.steady_state_temperature(loss)
                temps_vs_load.append(T)
                derating_vs_load.append(self.thermal_model.derating_factor(T, max_temp))

            ax2.plot(load_factors * 100, temps_vs_load, 'r-', linewidth=2, label='Temperature')
            ax2.axhline(max_temp, color='k', linestyle='--', label='Max Temp')
            ax2.set_xlabel('Load (%)')
            ax2.set_ylabel('Temperature (°C)', color='r')
            ax2.tick_params(axis='y', labelcolor='r')
            ax2.grid(True, alpha=0.3)

            ax2_twin = ax2.twinx()
            ax2_twin.plot(load_factors * 100, np.array(derating_vs_load) * 100, 'b-',
                         linewidth=2, label='Derating')
            ax2_twin.set_ylabel('Derating Factor (%)', color='b')
            ax2_twin.tick_params(axis='y', labelcolor='b')

            ax2.set_title('Temperature & Derating vs Load')
            ax2.legend(loc='upper left')
            ax2_twin.legend(loc='upper right')

            self.fig_thermal.tight_layout()
            self.canvas_thermal.draw()

            self.status_label.config(text="Thermal analysis complete")

        except Exception as e:
            messagebox.showerror("Thermal Analysis Error", f"Error:\n{str(e)}")
            self.status_label.config(text="Error")

    def analyze_mechanical_stress(self):
        """Analyze mechanical stress"""
        self.status_label.config(text="Analyzing mechanical stress...")
        self.root.update()

        try:
            # Get torque at operating point
            results = self.calculator.calculate_currents_and_power(self.params.slip)
            torque = results['torque']

            # Calculate shaft stress
            shaft_dia = self.shaft_dia_var.get()
            stress = self.mechanical_model.calculate_shaft_stress(torque, shaft_dia)

            # Display results
            self.mechanical_results_text.delete('1.0', tk.END)
            output = "MECHANICAL STRESS ANALYSIS:\n"
            output += "="*70 + "\n"
            output += f"Operating Torque:          {torque:8.4f} N·m\n"
            output += f"Shaft Diameter:            {shaft_dia*1000:8.2f} mm\n"
            output += f"Torsional Stress:          {stress['torsional_stress']/1e6:8.2f} MPa\n"
            output += f"Max Shear Stress:          {stress['max_shear_stress']/1e6:8.2f} MPa\n"
            output += f"Bearing Load:              {stress['bearing_load']:8.2f} N\n\n"

            # Material limits (example: steel shaft)
            yield_stress = 250e6  # Pa (mild steel)
            safety_factor = stress['max_shear_stress'] / (yield_stress / 2) if stress['max_shear_stress'] > 0 else 0

            output += f"Yield Stress (steel):      {yield_stress/1e6:8.2f} MPa\n"
            output += f"Safety Factor:             {1/safety_factor if safety_factor > 0 else float('inf'):8.2f}\n\n"

            if safety_factor > 0.5:
                output += "WARNING: Shaft stress exceeds 50% of yield stress!\n"
            else:
                output += "Shaft stress within acceptable limits.\n"

            self.mechanical_results_text.insert('1.0', output)

            # Plot stress distribution
            self.fig_mechanical.clear()

            ax1 = self.fig_mechanical.add_subplot(121)
            ax2 = self.fig_mechanical.add_subplot(122)

            # Stress vs torque
            torque_range = np.linspace(0, torque * 2, 50)
            stress_range = []

            for T in torque_range:
                s = self.mechanical_model.calculate_shaft_stress(T, shaft_dia)
                stress_range.append(s['torsional_stress'] / 1e6)

            ax1.plot(torque_range, stress_range, 'b-', linewidth=2)
            ax1.axvline(torque, color='r', linestyle='--', label='Operating Point')
            ax1.axhline(yield_stress / 2e6, color='k', linestyle='--', label='Yield Limit (shear)')
            ax1.set_xlabel('Torque (N·m)')
            ax1.set_ylabel('Torsional Stress (MPa)')
            ax1.set_title('Stress vs Torque')
            ax1.grid(True, alpha=0.3)
            ax1.legend()

            # Stress vs diameter
            dia_range = np.linspace(shaft_dia * 0.5, shaft_dia * 1.5, 50)
            stress_vs_dia = []

            for d in dia_range:
                s = self.mechanical_model.calculate_shaft_stress(torque, d)
                stress_vs_dia.append(s['torsional_stress'] / 1e6)

            ax2.plot(dia_range * 1000, stress_vs_dia, 'g-', linewidth=2)
            ax2.axvline(shaft_dia * 1000, color='r', linestyle='--', label='Current Diameter')
            ax2.axhline(yield_stress / 2e6, color='k', linestyle='--', label='Yield Limit')
            ax2.set_xlabel('Shaft Diameter (mm)')
            ax2.set_ylabel('Torsional Stress (MPa)')
            ax2.set_title('Stress vs Shaft Diameter')
            ax2.grid(True, alpha=0.3)
            ax2.legend()

            self.fig_mechanical.tight_layout()
            self.canvas_mechanical.draw()

            self.status_label.config(text="Mechanical analysis complete")

        except Exception as e:
            messagebox.showerror("Mechanical Analysis Error", f"Error:\n{str(e)}")
            self.status_label.config(text="Error")

    def calculate_economics(self):
        """Calculate economic analysis"""
        self.status_label.config(text="Calculating economics...")
        self.root.update()

        try:
            hours_per_year = self.hours_per_year_var.get()
            ref_efficiency = self.ref_efficiency_var.get()

            # Get current efficiency
            results = self.calculator.calculate_currents_and_power(self.params.slip)
            efficiency = results['efficiency']
            power_input_kw = results['Pin'] / 1000.0

            # Calculate costs
            costs = self.economic_model.calculate_operating_costs(power_input_kw, hours_per_year)
            comparison = self.economic_model.efficiency_comparison(efficiency, ref_efficiency)

            # Display results
            self.economic_results_text.delete('1.0', tk.END)
            output = "ECONOMIC ANALYSIS:\n"
            output += "="*70 + "\n\n"

            output += "OPERATING COSTS:\n"
            output += "-"*70 + "\n"
            output += f"  Operating Hours/Year:      {hours_per_year:10.0f} hours\n"
            output += f"  Electricity Cost:          ${self.params.electricity_cost:10.3f} /kWh\n"
            output += f"  Input Power:               {power_input_kw:10.3f} kW\n"
            output += f"  Annual Energy Cost:        ${costs['energy_cost_annual']:10.2f}\n"
            output += f"  Annual Maintenance:        ${costs['maintenance_cost_annual']:10.2f}\n"
            output += f"  Total Annual Cost:         ${costs['total_cost_annual']:10.2f}\n"
            output += f"  Cost per Hour:             ${costs['cost_per_hour']:10.4f}\n"
            output += f"  15-Year Lifetime Cost:     ${costs['total_lifetime_cost']:10.2f}\n\n"

            output += "EFFICIENCY COMPARISON:\n"
            output += "-"*70 + "\n"
            output += f"  Actual Efficiency:         {efficiency:10.2f} %\n"
            output += f"  Reference Efficiency:      {ref_efficiency:10.2f} %\n"
            output += f"  Actual Energy (annual):    {comparison['energy_actual_kwh']:10.2f} kWh\n"
            output += f"  Reference Energy (annual): {comparison['energy_reference_kwh']:10.2f} kWh\n"
            output += f"  Actual Cost (annual):      ${comparison['cost_actual']:10.2f}\n"
            output += f"  Reference Cost (annual):   ${comparison['cost_reference']:10.2f}\n"

            if comparison['annual_savings'] > 0:
                output += f"  Annual Savings:            ${comparison['annual_savings']:10.2f} "
                output += f"({comparison['savings_percentage']:.1f}%)\n"
                output += "\n  Motor is MORE efficient than reference!\n"
            else:
                output += f"  Additional Cost:           ${-comparison['annual_savings']:10.2f} "
                output += f"({-comparison['savings_percentage']:.1f}%)\n"
                output += "\n  Motor is LESS efficient than reference.\n"

            output += "\n" + "="*70 + "\n"

            self.economic_results_text.insert('1.0', output)

            self.status_label.config(text="Economic analysis complete")

        except Exception as e:
            messagebox.showerror("Economic Analysis Error", f"Error:\n{str(e)}")
            self.status_label.config(text="Error")

    def start_simulation(self):
        """Start simulation"""
        self.is_running = True
        self.status_label.config(text="Simulation running...")
        messagebox.showinfo("Simulation", "Continuous simulation mode activated.\nUse transient analysis in Dynamic Simulation tab.")

    def stop_simulation(self):
        """Stop simulation"""
        self.is_running = False
        self.status_label.config(text="Simulation stopped")

    def reset_simulation(self):
        """Reset simulation"""
        self.is_running = False
        self.simulation_time = 0.0
        self.time_history = []
        self.speed_history = []
        self.torque_history = []
        self.temp_history = []
        self.status_label.config(text="Simulation reset")

    def reset_parameters(self):
        """Reset all parameters to default"""
        self.params = MotorParameters()

        # Update all sliders
        for param_name, slider in self.sliders.items():
            value = getattr(self.params, param_name)
            slider.set(value)

        self.calculate_and_display()
        messagebox.showinfo("Reset", "Parameters reset to default values")

    def on_window_resize(self, event):
        """Handle window resize event"""
        # Auto-scale is handled by pack with expand=True
        pass

    def show_about(self):
        """Show about dialog"""
        about_text = """Advanced Single-Phase Induction Motor Simulator

Multi-Physics Simulation Platform

Features:
• Electromagnetic circuit analysis
• Dynamic simulation with RK45/Euler solvers
• Thermal analysis and derating
• Mechanical stress analysis
• Economic analysis
• Detailed loss breakdown
• Real-time visualization

Developed for electrical engineering education and analysis
"""
        messagebox.showinfo("About", about_text)


def main():
    """Main function"""
    root = tk.Tk()
    app = AdvancedMotorSimulatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    # First, solve the specific problem
    print("="*80)
    print("SOLVING SPECIFIC MOTOR PROBLEM")
    print("="*80)
    print("\nGiven:")
    print("  - Power: 185 W")
    print("  - Poles: 4")
    print("  - Voltage: 110 V")
    print("  - Frequency: 50 Hz")
    print("  - R1 = 1.86 Ω, X1 = 2.56 Ω")
    print("  - Xφ = 53.5 Ω")
    print("  - R2 = 3.56 Ω, X2 = 2.56 Ω")
    print("  - Core loss = 3.5 W")
    print("  - Friction & windage loss = 13.5 W")
    print("  - Slip = 0.05")
    print("\nCalculating...\n")

    # Create calculator with given parameters
    params = MotorParameters()
    calculator = MotorCalculations(params)

    # Calculate results
    results = calculator.calculate_currents_and_power(params.slip)

    # Display answer
    print("RESULTS:")
    print("-"*80)
    print(f"Mechanical Power Output = {results['Pout']:.2f} W")
    print(f"Efficiency = {results['efficiency']:.2f} %")
    print(f"Torque = {results['torque']:.4f} N·m")
    print(f"Speed = {results['speed_rpm']:.2f} rpm")
    print("-"*80)
    print("\nLaunching GUI application...\n")

    # Launch GUI
    main()
