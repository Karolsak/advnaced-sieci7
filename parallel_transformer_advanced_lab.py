#!/usr/bin/env python3
"""
Advanced Parallel Transformer Analysis and Multi-Physics Simulation Lab
Features:
- Parallel transformer load sharing analysis
- Multi-physics simulation (electromagnetic-thermal-mechanical)
- Real-time ODE solvers (RK45, Euler)
- Dynamic visualization
- Economic analysis
- Advanced thermal and derating analysis
- Detailed loss breakdown
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


class ParallelTransformerAnalysis:
    """Calculates load sharing between parallel transformers"""

    def __init__(self):
        self.reset_parameters()

    def reset_parameters(self):
        """Reset to default problem values"""
        # Problem 7 default values
        self.v_oc_primary = 11000  # V
        self.v_oc_secondary = 13300  # V
        self.total_load_current = 750  # A
        self.load_pf = 0.8  # lagging

        # Transformer A short circuit test
        self.vsc_a = 200  # V
        self.isc_a = 400  # A
        self.psc_a = 15000  # W (15 kW)

        # Transformer B short circuit test
        self.vsc_b = 100  # V
        self.isc_b = 400  # A
        self.psc_b = 20000  # W (20 kW)

        self.results = {}

    def calculate_impedances(self):
        """Calculate equivalent impedances from short circuit tests"""
        # Transformer A
        z_a = self.vsc_a / self.isc_a  # Impedance magnitude
        r_a = self.psc_a / (self.isc_a ** 2)  # Resistance
        x_a = np.sqrt(z_a**2 - r_a**2)  # Reactance

        # Transformer B
        z_b = self.vsc_b / self.isc_b
        r_b = self.psc_b / (self.isc_b ** 2)
        x_b = np.sqrt(z_b**2 - r_b**2)

        return {
            'z_a': z_a, 'r_a': r_a, 'x_a': x_a,
            'z_b': z_b, 'r_b': r_b, 'x_b': x_b
        }

    def calculate_load_sharing(self):
        """Calculate current and power sharing between transformers"""
        imp = self.calculate_impedances()

        # Load angle
        load_angle = np.arccos(self.load_pf)

        # Total load current (complex)
        i_total = self.total_load_current * np.exp(-1j * load_angle)

        # Complex impedances
        z_a_complex = imp['r_a'] + 1j * imp['x_a']
        z_b_complex = imp['r_b'] + 1j * imp['x_b']

        # Equivalent impedance
        z_eq = (z_a_complex * z_b_complex) / (z_a_complex + z_b_complex)

        # Voltage drop
        v_drop = i_total * z_eq

        # Secondary voltage (assuming nominal voltage)
        v2 = self.v_oc_secondary - v_drop
        v2_magnitude = abs(v2)

        # Individual transformer currents
        i_a = v_drop / z_a_complex
        i_b = v_drop / z_b_complex

        # Power factors
        pf_a = np.cos(np.angle(i_a) - np.angle(v2))
        pf_b = np.cos(np.angle(i_b) - np.angle(v2))

        # Apparent power
        s_a = v2_magnitude * abs(i_a)
        s_b = v2_magnitude * abs(i_b)

        # Real power
        p_a = s_a * pf_a
        p_b = s_b * pf_b

        self.results = {
            'impedances': imp,
            'v2_magnitude': v2_magnitude,
            'i_a_magnitude': abs(i_a),
            'i_b_magnitude': abs(i_b),
            'i_a_angle': np.degrees(np.angle(i_a)),
            'i_b_angle': np.degrees(np.angle(i_b)),
            'pf_a': pf_a,
            'pf_b': pf_b,
            's_a': s_a,
            's_b': s_b,
            'p_a': p_a,
            'p_b': p_b,
            'v_drop': abs(v_drop),
            'z_eq': abs(z_eq)
        }

        return self.results


class MultiPhysicsSimulator:
    """Multi-physics simulation engine"""

    def __init__(self):
        self.time_vector = []
        self.state_history = []
        self.thermal_history = []
        self.mechanical_history = []
        self.loss_history = []
        self.running = False
        self.solver_type = 'RK45'  # or 'Euler'

        # Physical parameters
        self.thermal_resistance = 0.5  # K/W
        self.thermal_capacitance = 2000  # J/K
        self.ambient_temp = 25  # °C
        self.shaft_inertia = 0.05  # kg⋅m²
        self.friction_coefficient = 0.01  # N⋅m⋅s

    def electromagnetic_model(self, t, y, v_in, load_current, frequency):
        """
        Electromagnetic differential equations
        State vector y = [i_primary, i_secondary, flux]
        """
        i_p, i_s, phi = y

        # Transformer parameters
        R_p = 0.5  # Primary resistance
        R_s = 0.3  # Secondary resistance
        L_p = 0.1  # Primary inductance
        L_s = 0.08  # Secondary inductance
        M = 0.09  # Mutual inductance

        # Voltage equations
        v_p = v_in * np.sin(2 * np.pi * frequency * t)

        # Differential equations
        di_p_dt = (v_p - R_p * i_p - M * i_s) / L_p
        di_s_dt = (M * i_p - R_s * i_s - load_current) / L_s
        dphi_dt = v_p - R_p * i_p

        return [di_p_dt, di_s_dt, dphi_dt]

    def thermal_model(self, T, P_loss):
        """
        Thermal differential equation
        dT/dt = (P_loss - (T - T_ambient)/R_th) / C_th
        """
        dT_dt = (P_loss - (T - self.ambient_temp) / self.thermal_resistance) / self.thermal_capacitance
        return dT_dt

    def mechanical_model(self, omega, torque_elec, torque_load):
        """
        Mechanical differential equation
        J * dω/dt = T_elec - T_load - B*ω
        """
        d_omega_dt = (torque_elec - torque_load - self.friction_coefficient * omega) / self.shaft_inertia
        return d_omega_dt

    def calculate_losses(self, i_primary, i_secondary, flux, frequency):
        """Calculate detailed loss breakdown"""
        # Copper losses
        R_p = 0.5
        R_s = 0.3
        copper_loss_p = R_p * i_primary**2
        copper_loss_s = R_s * i_secondary**2
        total_copper_loss = copper_loss_p + copper_loss_s

        # Iron losses (hysteresis + eddy current)
        k_h = 0.02  # Hysteresis coefficient
        k_e = 0.001  # Eddy current coefficient
        B_max = abs(flux) / 1000  # Flux density

        hysteresis_loss = k_h * frequency * B_max**2
        eddy_loss = k_e * frequency**2 * B_max**2
        total_iron_loss = hysteresis_loss + eddy_loss

        # Mechanical friction loss
        friction_loss = 50  # W (constant)

        # Stray load loss (approximation)
        stray_loss = 0.01 * (copper_loss_p + copper_loss_s)

        total_loss = total_copper_loss + total_iron_loss + friction_loss + stray_loss

        return {
            'copper_p': copper_loss_p,
            'copper_s': copper_loss_s,
            'hysteresis': hysteresis_loss,
            'eddy': eddy_loss,
            'friction': friction_loss,
            'stray': stray_loss,
            'total': total_loss
        }

    def run_simulation_rk45(self, duration, v_in, load_current, frequency):
        """Run simulation using RK45 solver"""
        self.time_vector = []
        self.state_history = []
        self.thermal_history = []
        self.mechanical_history = []
        self.loss_history = []

        # Initial conditions
        y0 = [0, 0, 0]  # i_p, i_s, flux
        T = self.ambient_temp
        omega = 0

        t_span = (0, duration)
        t_eval = np.linspace(0, duration, 1000)

        # Solve electromagnetic equations
        sol = solve_ivp(
            lambda t, y: self.electromagnetic_model(t, y, v_in, load_current, frequency),
            t_span, y0, t_eval=t_eval, method='RK45'
        )

        # Process results and calculate thermal/mechanical
        for i, t in enumerate(sol.t):
            i_p, i_s, phi = sol.y[:, i]

            # Calculate losses
            losses = self.calculate_losses(i_p, i_s, phi, frequency)

            # Update thermal state (simple Euler for thermal)
            if i > 0:
                dt = sol.t[i] - sol.t[i-1]
                dT_dt = self.thermal_model(T, losses['total'])
                T += dT_dt * dt

            # Calculate electromagnetic torque
            torque_elec = 0.9 * i_p * i_s  # Simplified model
            torque_load = 5  # N⋅m

            # Update mechanical state
            if i > 0:
                dt = sol.t[i] - sol.t[i-1]
                d_omega_dt = self.mechanical_model(omega, torque_elec, torque_load)
                omega += d_omega_dt * dt

            self.time_vector.append(t)
            self.state_history.append([i_p, i_s, phi])
            self.thermal_history.append(T)
            self.mechanical_history.append([omega, torque_elec])
            self.loss_history.append(losses)

    def run_simulation_euler(self, duration, v_in, load_current, frequency, dt=0.001):
        """Run simulation using Euler method"""
        self.time_vector = []
        self.state_history = []
        self.thermal_history = []
        self.mechanical_history = []
        self.loss_history = []

        # Initial conditions
        i_p, i_s, phi = 0, 0, 0
        T = self.ambient_temp
        omega = 0

        t = 0
        while t <= duration:
            # Electromagnetic state derivatives
            dy_dt = self.electromagnetic_model(t, [i_p, i_s, phi], v_in, load_current, frequency)

            # Update electromagnetic states
            i_p += dy_dt[0] * dt
            i_s += dy_dt[1] * dt
            phi += dy_dt[2] * dt

            # Calculate losses
            losses = self.calculate_losses(i_p, i_s, phi, frequency)

            # Update thermal state
            dT_dt = self.thermal_model(T, losses['total'])
            T += dT_dt * dt

            # Calculate electromagnetic torque
            torque_elec = 0.9 * i_p * i_s
            torque_load = 5  # N⋅m

            # Update mechanical state
            d_omega_dt = self.mechanical_model(omega, torque_elec, torque_load)
            omega += d_omega_dt * dt

            # Store results
            self.time_vector.append(t)
            self.state_history.append([i_p, i_s, phi])
            self.thermal_history.append(T)
            self.mechanical_history.append([omega, torque_elec])
            self.loss_history.append(losses)

            t += dt

    def run_simulation(self, duration, v_in, load_current, frequency):
        """Run simulation based on selected solver"""
        if self.solver_type == 'RK45':
            self.run_simulation_rk45(duration, v_in, load_current, frequency)
        else:
            self.run_simulation_euler(duration, v_in, load_current, frequency)


class EconomicAnalyzer:
    """Economic analysis module"""

    def __init__(self):
        self.electricity_cost = 0.12  # $/kWh
        self.operating_hours = 8760  # hours/year
        self.transformer_cost = 50000  # $
        self.maintenance_cost_annual = 2000  # $/year
        self.lifetime_years = 25
        self.discount_rate = 0.05

    def calculate_energy_cost(self, power_loss_kw):
        """Calculate annual energy cost"""
        annual_energy_loss = power_loss_kw * self.operating_hours
        annual_cost = annual_energy_loss * self.electricity_cost
        return annual_cost

    def calculate_npv(self, annual_loss_kw):
        """Calculate Net Present Value"""
        annual_cost = self.calculate_energy_cost(annual_loss_kw) + self.maintenance_cost_annual

        # NPV calculation
        npv = -self.transformer_cost
        for year in range(1, self.lifetime_years + 1):
            npv -= annual_cost / ((1 + self.discount_rate) ** year)

        return npv

    def calculate_payback_period(self, annual_savings):
        """Calculate simple payback period"""
        if annual_savings <= 0:
            return float('inf')
        return self.transformer_cost / annual_savings


class AdvancedTransformerLab(tk.Tk):
    """Main application class"""

    def __init__(self):
        super().__init__()

        self.title("Advanced Parallel Transformer & Multi-Physics Simulation Lab")
        self.geometry("1400x900")

        # Initialize components
        self.transformer_analysis = ParallelTransformerAnalysis()
        self.simulator = MultiPhysicsSimulator()
        self.economic_analyzer = EconomicAnalyzer()

        # Simulation control
        self.simulation_running = False
        self.animation_thread = None

        # Create GUI
        self.create_menu()
        self.create_notebook()

        # Bind resize event
        self.bind('<Configure>', self.on_window_resize)

        # Auto-scale flag
        self.auto_scale = True

    def create_menu(self):
        """Create main menu"""
        menubar = tk.Menu(self)
        self.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Results", command=self.save_results)
        file_menu.add_command(label="Load Parameters", command=self.load_parameters)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.quit)

        # Simulation menu
        sim_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        sim_menu.add_command(label="Run", command=self.start_simulation)
        sim_menu.add_command(label="Stop", command=self.stop_simulation)
        sim_menu.add_command(label="Reset", command=self.reset_simulation)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def create_notebook(self):
        """Create tabbed interface"""
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_parallel_transformer_tab()
        self.create_multiphysics_tab()
        self.create_visualization_tab()
        self.create_economic_tab()
        self.create_advanced_controls_tab()

    def create_parallel_transformer_tab(self):
        """Tab 1: Parallel Transformer Analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Parallel Transformers")

        # Left frame - inputs
        left_frame = ttk.LabelFrame(tab, text="Input Parameters")
        left_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')

        # Load parameters
        ttk.Label(left_frame, text="Total Load Current (A):").grid(row=0, column=0, sticky='w', padx=5, pady=5)
        self.load_current_var = tk.DoubleVar(value=750)
        ttk.Entry(left_frame, textvariable=self.load_current_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(left_frame, text="Load Power Factor:").grid(row=1, column=0, sticky='w', padx=5, pady=5)
        self.load_pf_var = tk.DoubleVar(value=0.8)
        ttk.Scale(left_frame, from_=0.5, to=1.0, variable=self.load_pf_var, orient='horizontal', length=200).grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(left_frame, textvariable=self.load_pf_var).grid(row=1, column=2, padx=5, pady=5)

        # Transformer A parameters
        ttk.Label(left_frame, text="Transformer A - Vsc (V):").grid(row=2, column=0, sticky='w', padx=5, pady=5)
        self.vsc_a_var = tk.DoubleVar(value=200)
        ttk.Entry(left_frame, textvariable=self.vsc_a_var, width=15).grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(left_frame, text="Transformer A - Isc (A):").grid(row=3, column=0, sticky='w', padx=5, pady=5)
        self.isc_a_var = tk.DoubleVar(value=400)
        ttk.Entry(left_frame, textvariable=self.isc_a_var, width=15).grid(row=3, column=1, padx=5, pady=5)

        ttk.Label(left_frame, text="Transformer A - Psc (kW):").grid(row=4, column=0, sticky='w', padx=5, pady=5)
        self.psc_a_var = tk.DoubleVar(value=15)
        ttk.Entry(left_frame, textvariable=self.psc_a_var, width=15).grid(row=4, column=1, padx=5, pady=5)

        # Transformer B parameters
        ttk.Label(left_frame, text="Transformer B - Vsc (V):").grid(row=5, column=0, sticky='w', padx=5, pady=5)
        self.vsc_b_var = tk.DoubleVar(value=100)
        ttk.Entry(left_frame, textvariable=self.vsc_b_var, width=15).grid(row=5, column=1, padx=5, pady=5)

        ttk.Label(left_frame, text="Transformer B - Isc (A):").grid(row=6, column=0, sticky='w', padx=5, pady=5)
        self.isc_b_var = tk.DoubleVar(value=400)
        ttk.Entry(left_frame, textvariable=self.isc_b_var, width=15).grid(row=6, column=1, padx=5, pady=5)

        ttk.Label(left_frame, text="Transformer B - Psc (kW):").grid(row=7, column=0, sticky='w', padx=5, pady=5)
        self.psc_b_var = tk.DoubleVar(value=20)
        ttk.Entry(left_frame, textvariable=self.psc_b_var, width=15).grid(row=7, column=1, padx=5, pady=5)

        # Calculate button
        ttk.Button(left_frame, text="Calculate Load Sharing", command=self.calculate_parallel_transformers).grid(row=8, column=0, columnspan=3, pady=10)

        # Right frame - results
        right_frame = ttk.LabelFrame(tab, text="Results")
        right_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nsew')

        self.results_text = tk.Text(right_frame, width=60, height=30, font=('Courier', 10))
        self.results_text.pack(padx=5, pady=5, fill='both', expand=True)

        scrollbar = ttk.Scrollbar(right_frame, command=self.results_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Configure grid weights
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_columnconfigure(1, weight=2)
        tab.grid_rowconfigure(0, weight=1)

    def create_multiphysics_tab(self):
        """Tab 2: Multi-Physics Simulation"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Multi-Physics Simulation")

        # Control frame
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls")
        control_frame.grid(row=0, column=0, padx=10, pady=10, sticky='ew', columnspan=2)

        ttk.Label(control_frame, text="Input Voltage (V):").grid(row=0, column=0, padx=5, pady=5)
        self.v_in_var = tk.DoubleVar(value=230)
        ttk.Scale(control_frame, from_=100, to=400, variable=self.v_in_var, orient='horizontal', length=200).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(control_frame, textvariable=self.v_in_var).grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(control_frame, text="Load Current (A):").grid(row=1, column=0, padx=5, pady=5)
        self.sim_load_current_var = tk.DoubleVar(value=10)
        ttk.Scale(control_frame, from_=1, to=50, variable=self.sim_load_current_var, orient='horizontal', length=200).grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(control_frame, textvariable=self.sim_load_current_var).grid(row=1, column=2, padx=5, pady=5)

        ttk.Label(control_frame, text="Frequency (Hz):").grid(row=2, column=0, padx=5, pady=5)
        self.frequency_var = tk.DoubleVar(value=50)
        ttk.Entry(control_frame, textvariable=self.frequency_var, width=15).grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(control_frame, text="Duration (s):").grid(row=3, column=0, padx=5, pady=5)
        self.duration_var = tk.DoubleVar(value=0.1)
        ttk.Entry(control_frame, textvariable=self.duration_var, width=15).grid(row=3, column=1, padx=5, pady=5)

        ttk.Label(control_frame, text="Solver:").grid(row=4, column=0, padx=5, pady=5)
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(control_frame, text="RK45", variable=self.solver_var, value='RK45').grid(row=4, column=1, sticky='w')
        ttk.Radiobutton(control_frame, text="Euler", variable=self.solver_var, value='Euler').grid(row=4, column=2, sticky='w')

        # Buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=5, column=0, columnspan=3, pady=10)
        ttk.Button(button_frame, text="Start", command=self.start_simulation).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Stop", command=self.stop_simulation).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Reset", command=self.reset_simulation).pack(side='left', padx=5)

        # Results frame
        results_frame = ttk.LabelFrame(tab, text="Simulation Results")
        results_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky='nsew')

        self.sim_results_text = tk.Text(results_frame, height=15, font=('Courier', 9))
        self.sim_results_text.pack(fill='both', expand=True, padx=5, pady=5)

        # Configure grid weights
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)

    def create_visualization_tab(self):
        """Tab 3: Dynamic Visualization"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Visualization")

        # Create matplotlib figure with subplots
        self.fig = Figure(figsize=(12, 8))

        # Electromagnetic plots
        self.ax1 = self.fig.add_subplot(3, 2, 1)
        self.ax1.set_title('Primary Current vs Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Current (A)')
        self.ax1.grid(True)

        self.ax2 = self.fig.add_subplot(3, 2, 2)
        self.ax2.set_title('Secondary Current vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A)')
        self.ax2.grid(True)

        # Thermal plot
        self.ax3 = self.fig.add_subplot(3, 2, 3)
        self.ax3.set_title('Temperature vs Time')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Temperature (°C)')
        self.ax3.grid(True)

        # Mechanical plot
        self.ax4 = self.fig.add_subplot(3, 2, 4)
        self.ax4.set_title('Angular Velocity vs Time')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('ω (rad/s)')
        self.ax4.grid(True)

        # Loss breakdown pie chart
        self.ax5 = self.fig.add_subplot(3, 2, 5)
        self.ax5.set_title('Loss Breakdown')

        # Power flow
        self.ax6 = self.fig.add_subplot(3, 2, 6)
        self.ax6.set_title('Total Losses vs Time')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Power (W)')
        self.ax6.grid(True)

        self.fig.tight_layout()

        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, tab)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

    def create_economic_tab(self):
        """Tab 4: Economic Analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Economic Analysis")

        # Input frame
        input_frame = ttk.LabelFrame(tab, text="Economic Parameters")
        input_frame.grid(row=0, column=0, padx=10, pady=10, sticky='ew')

        ttk.Label(input_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w', padx=5, pady=5)
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(input_frame, textvariable=self.elec_cost_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="Operating Hours/Year:").grid(row=1, column=0, sticky='w', padx=5, pady=5)
        self.op_hours_var = tk.DoubleVar(value=8760)
        ttk.Entry(input_frame, textvariable=self.op_hours_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="Transformer Cost ($):").grid(row=2, column=0, sticky='w', padx=5, pady=5)
        self.trans_cost_var = tk.DoubleVar(value=50000)
        ttk.Entry(input_frame, textvariable=self.trans_cost_var, width=15).grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="Annual Maintenance ($):").grid(row=3, column=0, sticky='w', padx=5, pady=5)
        self.maint_cost_var = tk.DoubleVar(value=2000)
        ttk.Entry(input_frame, textvariable=self.maint_cost_var, width=15).grid(row=3, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="Lifetime (years):").grid(row=4, column=0, sticky='w', padx=5, pady=5)
        self.lifetime_var = tk.DoubleVar(value=25)
        ttk.Entry(input_frame, textvariable=self.lifetime_var, width=15).grid(row=4, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="Discount Rate:").grid(row=5, column=0, sticky='w', padx=5, pady=5)
        self.discount_var = tk.DoubleVar(value=0.05)
        ttk.Entry(input_frame, textvariable=self.discount_var, width=15).grid(row=5, column=1, padx=5, pady=5)

        ttk.Button(input_frame, text="Calculate Economics", command=self.calculate_economics).grid(row=6, column=0, columnspan=2, pady=10)

        # Results frame
        results_frame = ttk.LabelFrame(tab, text="Economic Analysis Results")
        results_frame.grid(row=1, column=0, padx=10, pady=10, sticky='nsew')

        self.econ_results_text = tk.Text(results_frame, height=20, font=('Courier', 10))
        self.econ_results_text.pack(fill='both', expand=True, padx=5, pady=5)

        # Configure grid weights
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)

    def create_advanced_controls_tab(self):
        """Tab 5: Advanced Controls & Thermal Analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Advanced Controls")

        # Thermal parameters
        thermal_frame = ttk.LabelFrame(tab, text="Thermal Parameters")
        thermal_frame.grid(row=0, column=0, padx=10, pady=10, sticky='ew')

        ttk.Label(thermal_frame, text="Thermal Resistance (K/W):").grid(row=0, column=0, sticky='w', padx=5, pady=5)
        self.thermal_r_var = tk.DoubleVar(value=0.5)
        ttk.Scale(thermal_frame, from_=0.1, to=2.0, variable=self.thermal_r_var, orient='horizontal', length=200).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(thermal_frame, textvariable=self.thermal_r_var).grid(row=0, column=2)

        ttk.Label(thermal_frame, text="Thermal Capacitance (J/K):").grid(row=1, column=0, sticky='w', padx=5, pady=5)
        self.thermal_c_var = tk.DoubleVar(value=2000)
        ttk.Scale(thermal_frame, from_=500, to=5000, variable=self.thermal_c_var, orient='horizontal', length=200).grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(thermal_frame, textvariable=self.thermal_c_var).grid(row=1, column=2)

        ttk.Label(thermal_frame, text="Ambient Temperature (°C):").grid(row=2, column=0, sticky='w', padx=5, pady=5)
        self.ambient_temp_var = tk.DoubleVar(value=25)
        ttk.Scale(thermal_frame, from_=0, to=50, variable=self.ambient_temp_var, orient='horizontal', length=200).grid(row=2, column=1, padx=5, pady=5)
        ttk.Label(thermal_frame, textvariable=self.ambient_temp_var).grid(row=2, column=2)

        # Mechanical parameters
        mech_frame = ttk.LabelFrame(tab, text="Mechanical Parameters")
        mech_frame.grid(row=1, column=0, padx=10, pady=10, sticky='ew')

        ttk.Label(mech_frame, text="Shaft Inertia (kg⋅m²):").grid(row=0, column=0, sticky='w', padx=5, pady=5)
        self.inertia_var = tk.DoubleVar(value=0.05)
        ttk.Entry(mech_frame, textvariable=self.inertia_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(mech_frame, text="Friction Coefficient (N⋅m⋅s):").grid(row=1, column=0, sticky='w', padx=5, pady=5)
        self.friction_var = tk.DoubleVar(value=0.01)
        ttk.Entry(mech_frame, textvariable=self.friction_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        # Derating analysis
        derating_frame = ttk.LabelFrame(tab, text="Derating Analysis")
        derating_frame.grid(row=2, column=0, padx=10, pady=10, sticky='nsew')

        self.derating_text = tk.Text(derating_frame, height=15, font=('Courier', 10))
        self.derating_text.pack(fill='both', expand=True, padx=5, pady=5)

        ttk.Button(tab, text="Calculate Derating", command=self.calculate_derating).grid(row=3, column=0, pady=10)

        # Configure grid weights
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(2, weight=1)

    def calculate_parallel_transformers(self):
        """Calculate and display parallel transformer results"""
        # Update transformer analysis parameters
        self.transformer_analysis.total_load_current = self.load_current_var.get()
        self.transformer_analysis.load_pf = self.load_pf_var.get()
        self.transformer_analysis.vsc_a = self.vsc_a_var.get()
        self.transformer_analysis.isc_a = self.isc_a_var.get()
        self.transformer_analysis.psc_a = self.psc_a_var.get() * 1000  # Convert to W
        self.transformer_analysis.vsc_b = self.vsc_b_var.get()
        self.transformer_analysis.isc_b = self.isc_b_var.get()
        self.transformer_analysis.psc_b = self.psc_b_var.get() * 1000  # Convert to W

        # Calculate
        results = self.transformer_analysis.calculate_load_sharing()

        # Display results
        self.results_text.delete('1.0', tk.END)
        output = "="*70 + "\n"
        output += "PARALLEL TRANSFORMER ANALYSIS RESULTS\n"
        output += "="*70 + "\n\n"

        output += "IMPEDANCES:\n"
        output += "-"*70 + "\n"
        output += f"Transformer A:\n"
        output += f"  Impedance (Z_A):      {results['impedances']['z_a']:.4f} Ω\n"
        output += f"  Resistance (R_A):     {results['impedances']['r_a']:.4f} Ω\n"
        output += f"  Reactance (X_A):      {results['impedances']['x_a']:.4f} Ω\n\n"

        output += f"Transformer B:\n"
        output += f"  Impedance (Z_B):      {results['impedances']['z_b']:.4f} Ω\n"
        output += f"  Resistance (R_B):     {results['impedances']['r_b']:.4f} Ω\n"
        output += f"  Reactance (X_B):      {results['impedances']['x_b']:.4f} Ω\n\n"

        output += f"Equivalent Impedance:   {results['z_eq']:.4f} Ω\n\n"

        output += "VOLTAGE:\n"
        output += "-"*70 + "\n"
        output += f"Secondary Voltage:      {results['v2_magnitude']:.2f} V\n"
        output += f"Voltage Drop:           {results['v_drop']:.2f} V\n\n"

        output += "CURRENT DISTRIBUTION:\n"
        output += "-"*70 + "\n"
        output += f"Transformer A Current:  {results['i_a_magnitude']:.2f} A ∠{results['i_a_angle']:.2f}°\n"
        output += f"Transformer B Current:  {results['i_b_magnitude']:.2f} A ∠{results['i_b_angle']:.2f}°\n"
        output += f"Total Current:          {self.transformer_analysis.total_load_current:.2f} A\n\n"

        output += "POWER FACTOR:\n"
        output += "-"*70 + "\n"
        output += f"Transformer A P.F.:     {results['pf_a']:.4f} {'lagging' if results['pf_a'] > 0 else 'leading'}\n"
        output += f"Transformer B P.F.:     {results['pf_b']:.4f} {'lagging' if results['pf_b'] > 0 else 'leading'}\n"
        output += f"Load P.F.:              {self.transformer_analysis.load_pf:.4f} lagging\n\n"

        output += "POWER OUTPUT:\n"
        output += "-"*70 + "\n"
        output += f"Transformer A:\n"
        output += f"  Apparent Power (S_A): {results['s_a']/1000:.2f} kVA\n"
        output += f"  Real Power (P_A):     {results['p_a']/1000:.2f} kW\n\n"

        output += f"Transformer B:\n"
        output += f"  Apparent Power (S_B): {results['s_b']/1000:.2f} kVA\n"
        output += f"  Real Power (P_B):     {results['p_b']/1000:.2f} kW\n\n"

        output += f"Total Apparent Power:   {(results['s_a'] + results['s_b'])/1000:.2f} kVA\n"
        output += f"Total Real Power:       {(results['p_a'] + results['p_b'])/1000:.2f} kW\n\n"

        output += "LOAD SHARING PERCENTAGE:\n"
        output += "-"*70 + "\n"
        total_current = results['i_a_magnitude'] + results['i_b_magnitude']
        output += f"Transformer A:          {(results['i_a_magnitude']/total_current)*100:.2f}%\n"
        output += f"Transformer B:          {(results['i_b_magnitude']/total_current)*100:.2f}%\n"

        self.results_text.insert('1.0', output)

    def start_simulation(self):
        """Start multi-physics simulation"""
        if self.simulation_running:
            messagebox.showwarning("Warning", "Simulation already running!")
            return

        self.simulation_running = True

        # Update simulator parameters
        self.simulator.solver_type = self.solver_var.get()
        self.simulator.thermal_resistance = self.thermal_r_var.get()
        self.simulator.thermal_capacitance = self.thermal_c_var.get()
        self.simulator.ambient_temp = self.ambient_temp_var.get()
        self.simulator.shaft_inertia = self.inertia_var.get()
        self.simulator.friction_coefficient = self.friction_var.get()

        # Run simulation in thread
        self.animation_thread = threading.Thread(target=self.run_simulation_thread)
        self.animation_thread.daemon = True
        self.animation_thread.start()

    def run_simulation_thread(self):
        """Run simulation in background thread"""
        try:
            v_in = self.v_in_var.get()
            load_current = self.sim_load_current_var.get()
            frequency = self.frequency_var.get()
            duration = self.duration_var.get()

            # Run simulation
            self.simulator.run_simulation(duration, v_in, load_current, frequency)

            # Update visualization
            self.after(0, self.update_visualization)

            # Update results text
            self.after(0, self.update_simulation_results)

        except Exception as e:
            messagebox.showerror("Error", f"Simulation error: {str(e)}")
        finally:
            self.simulation_running = False

    def update_visualization(self):
        """Update all plots"""
        if not self.simulator.time_vector:
            return

        time = np.array(self.simulator.time_vector)
        states = np.array(self.simulator.state_history)
        thermal = np.array(self.simulator.thermal_history)
        mechanical = np.array(self.simulator.mechanical_history)

        # Clear all axes
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax6]:
            ax.clear()

        # Plot primary current
        self.ax1.plot(time, states[:, 0], 'b-', linewidth=1.5)
        self.ax1.set_title('Primary Current vs Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Current (A)')
        self.ax1.grid(True, alpha=0.3)

        # Plot secondary current
        self.ax2.plot(time, states[:, 1], 'r-', linewidth=1.5)
        self.ax2.set_title('Secondary Current vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A)')
        self.ax2.grid(True, alpha=0.3)

        # Plot temperature
        self.ax3.plot(time, thermal, 'g-', linewidth=1.5)
        self.ax3.axhline(y=self.ambient_temp_var.get(), color='k', linestyle='--', label='Ambient')
        self.ax3.set_title('Temperature vs Time')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Temperature (°C)')
        self.ax3.legend()
        self.ax3.grid(True, alpha=0.3)

        # Plot angular velocity
        self.ax4.plot(time, mechanical[:, 0], 'm-', linewidth=1.5)
        self.ax4.set_title('Angular Velocity vs Time')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('ω (rad/s)')
        self.ax4.grid(True, alpha=0.3)

        # Loss breakdown pie chart
        if self.simulator.loss_history:
            avg_losses = {
                'Copper (Primary)': np.mean([l['copper_p'] for l in self.simulator.loss_history]),
                'Copper (Secondary)': np.mean([l['copper_s'] for l in self.simulator.loss_history]),
                'Hysteresis': np.mean([l['hysteresis'] for l in self.simulator.loss_history]),
                'Eddy Current': np.mean([l['eddy'] for l in self.simulator.loss_history]),
                'Friction': np.mean([l['friction'] for l in self.simulator.loss_history]),
                'Stray': np.mean([l['stray'] for l in self.simulator.loss_history])
            }

            self.ax5.clear()
            self.ax5.pie(avg_losses.values(), labels=avg_losses.keys(), autopct='%1.1f%%', startangle=90)
            self.ax5.set_title('Average Loss Breakdown')

        # Plot total losses
        total_losses = [l['total'] for l in self.simulator.loss_history]
        self.ax6.plot(time, total_losses, 'orange', linewidth=1.5)
        self.ax6.set_title('Total Losses vs Time')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Power (W)')
        self.ax6.grid(True, alpha=0.3)

        self.fig.tight_layout()
        self.canvas.draw()

    def update_simulation_results(self):
        """Update simulation results text"""
        if not self.simulator.loss_history:
            return

        self.sim_results_text.delete('1.0', tk.END)
        output = "="*70 + "\n"
        output += "MULTI-PHYSICS SIMULATION RESULTS\n"
        output += "="*70 + "\n\n"

        output += f"Solver Method:          {self.simulator.solver_type}\n"
        output += f"Simulation Duration:    {self.duration_var.get()} s\n"
        output += f"Time Steps:             {len(self.simulator.time_vector)}\n\n"

        # Final values
        final_state = self.simulator.state_history[-1]
        final_temp = self.simulator.thermal_history[-1]
        final_mech = self.simulator.mechanical_history[-1]
        final_losses = self.simulator.loss_history[-1]

        output += "FINAL STATE:\n"
        output += "-"*70 + "\n"
        output += f"Primary Current (RMS):  {abs(final_state[0])/np.sqrt(2):.4f} A\n"
        output += f"Secondary Current (RMS):{abs(final_state[1])/np.sqrt(2):.4f} A\n"
        output += f"Flux Linkage:           {final_state[2]:.4f} Wb\n"
        output += f"Temperature:            {final_temp:.2f} °C\n"
        output += f"Angular Velocity:       {final_mech[0]:.4f} rad/s\n"
        output += f"Electromagnetic Torque: {final_mech[1]:.4f} N⋅m\n\n"

        output += "THERMAL ANALYSIS:\n"
        output += "-"*70 + "\n"
        max_temp = max(self.simulator.thermal_history)
        temp_rise = max_temp - self.ambient_temp_var.get()
        output += f"Maximum Temperature:    {max_temp:.2f} °C\n"
        output += f"Temperature Rise:       {temp_rise:.2f} K\n"
        output += f"Ambient Temperature:    {self.ambient_temp_var.get():.2f} °C\n"

        # Thermal rating
        if max_temp > 105:
            rating = "OVERHEATING - Reduce Load!"
        elif max_temp > 85:
            rating = "High Temperature - Monitor Closely"
        else:
            rating = "Normal Operation"
        output += f"Thermal Status:         {rating}\n\n"

        output += "LOSS BREAKDOWN (Average):\n"
        output += "-"*70 + "\n"
        avg_copper_p = np.mean([l['copper_p'] for l in self.simulator.loss_history])
        avg_copper_s = np.mean([l['copper_s'] for l in self.simulator.loss_history])
        avg_hysteresis = np.mean([l['hysteresis'] for l in self.simulator.loss_history])
        avg_eddy = np.mean([l['eddy'] for l in self.simulator.loss_history])
        avg_friction = np.mean([l['friction'] for l in self.simulator.loss_history])
        avg_stray = np.mean([l['stray'] for l in self.simulator.loss_history])
        avg_total = np.mean([l['total'] for l in self.simulator.loss_history])

        output += f"Copper Loss (Primary):  {avg_copper_p:.2f} W ({(avg_copper_p/avg_total)*100:.1f}%)\n"
        output += f"Copper Loss (Secondary):{avg_copper_s:.2f} W ({(avg_copper_s/avg_total)*100:.1f}%)\n"
        output += f"Hysteresis Loss:        {avg_hysteresis:.2f} W ({(avg_hysteresis/avg_total)*100:.1f}%)\n"
        output += f"Eddy Current Loss:      {avg_eddy:.2f} W ({(avg_eddy/avg_total)*100:.1f}%)\n"
        output += f"Friction Loss:          {avg_friction:.2f} W ({(avg_friction/avg_total)*100:.1f}%)\n"
        output += f"Stray Load Loss:        {avg_stray:.2f} W ({(avg_stray/avg_total)*100:.1f}%)\n"
        output += f"Total Average Loss:     {avg_total:.2f} W\n\n"

        output += "EFFICIENCY ANALYSIS:\n"
        output += "-"*70 + "\n"
        p_out = self.v_in_var.get() * self.sim_load_current_var.get() * 0.8  # Assume 0.8 pf
        p_in = p_out + avg_total
        efficiency = (p_out / p_in) * 100
        output += f"Output Power:           {p_out:.2f} W\n"
        output += f"Input Power:            {p_in:.2f} W\n"
        output += f"Efficiency:             {efficiency:.2f} %\n"

        self.sim_results_text.insert('1.0', output)

    def stop_simulation(self):
        """Stop running simulation"""
        self.simulation_running = False
        messagebox.showinfo("Info", "Simulation stopped")

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()

        # Clear plots
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            ax.clear()
        self.canvas.draw()

        # Clear results
        self.sim_results_text.delete('1.0', tk.END)

        # Reset simulator
        self.simulator = MultiPhysicsSimulator()

        messagebox.showinfo("Info", "Simulation reset")

    def calculate_economics(self):
        """Calculate economic analysis"""
        # Update economic analyzer parameters
        self.economic_analyzer.electricity_cost = self.elec_cost_var.get()
        self.economic_analyzer.operating_hours = self.op_hours_var.get()
        self.economic_analyzer.transformer_cost = self.trans_cost_var.get()
        self.economic_analyzer.maintenance_cost_annual = self.maint_cost_var.get()
        self.economic_analyzer.lifetime_years = int(self.lifetime_var.get())
        self.economic_analyzer.discount_rate = self.discount_var.get()

        # Get average loss from simulation if available
        if self.simulator.loss_history:
            avg_loss_w = np.mean([l['total'] for l in self.simulator.loss_history])
            avg_loss_kw = avg_loss_w / 1000
        else:
            avg_loss_kw = 1.0  # Default assumption

        # Calculate economics
        annual_energy_cost = self.economic_analyzer.calculate_energy_cost(avg_loss_kw)
        npv = self.economic_analyzer.calculate_npv(avg_loss_kw)

        # Display results
        self.econ_results_text.delete('1.0', tk.END)
        output = "="*70 + "\n"
        output += "ECONOMIC ANALYSIS RESULTS\n"
        output += "="*70 + "\n\n"

        output += "INPUT PARAMETERS:\n"
        output += "-"*70 + "\n"
        output += f"Electricity Cost:       ${self.elec_cost_var.get():.4f}/kWh\n"
        output += f"Operating Hours/Year:   {self.op_hours_var.get():.0f} hours\n"
        output += f"Transformer Cost:       ${self.trans_cost_var.get():,.2f}\n"
        output += f"Annual Maintenance:     ${self.maint_cost_var.get():,.2f}\n"
        output += f"Lifetime:               {int(self.lifetime_var.get())} years\n"
        output += f"Discount Rate:          {self.discount_var.get()*100:.2f}%\n\n"

        output += "LOSS ANALYSIS:\n"
        output += "-"*70 + "\n"
        output += f"Average Power Loss:     {avg_loss_kw:.3f} kW\n"
        output += f"Annual Energy Loss:     {avg_loss_kw * self.op_hours_var.get():.2f} kWh/year\n\n"

        output += "COST ANALYSIS:\n"
        output += "-"*70 + "\n"
        output += f"Annual Energy Cost:     ${annual_energy_cost:,.2f}/year\n"
        output += f"Annual Total Cost:      ${annual_energy_cost + self.maint_cost_var.get():,.2f}/year\n"

        total_lifetime_cost = (annual_energy_cost + self.maint_cost_var.get()) * self.lifetime_var.get()
        output += f"Lifetime Operating Cost:${total_lifetime_cost:,.2f}\n"
        output += f"Total Cost (Capital+Op):${self.trans_cost_var.get() + total_lifetime_cost:,.2f}\n\n"

        output += "NET PRESENT VALUE:\n"
        output += "-"*70 + "\n"
        output += f"NPV:                    ${npv:,.2f}\n\n"

        output += "COMPARATIVE ANALYSIS:\n"
        output += "-"*70 + "\n"
        # Compare with more efficient transformer (10% less loss)
        improved_loss_kw = avg_loss_kw * 0.9
        improved_annual_cost = self.economic_analyzer.calculate_energy_cost(improved_loss_kw)
        annual_savings = annual_energy_cost - improved_annual_cost

        output += f"If losses reduced by 10%:\n"
        output += f"  New Annual Energy Cost: ${improved_annual_cost:,.2f}/year\n"
        output += f"  Annual Savings:         ${annual_savings:,.2f}/year\n"
        output += f"  Lifetime Savings:       ${annual_savings * self.lifetime_var.get():,.2f}\n"

        if annual_savings > 0:
            payback = self.economic_analyzer.calculate_payback_period(annual_savings)
            output += f"  Payback Period:         {payback:.1f} years\n"

        output += "\n"
        output += "RECOMMENDATIONS:\n"
        output += "-"*70 + "\n"
        if avg_loss_kw > 2.0:
            output += "• High losses detected - consider upgrading to more efficient transformer\n"
        if annual_energy_cost > 5000:
            output += "• Annual energy costs are significant - energy efficiency improvements recommended\n"
        if npv < -100000:
            output += "• Poor economic performance - review operating conditions and maintenance\n"
        output += "• Regular maintenance reduces long-term costs\n"
        output += "• Monitor load patterns for optimization opportunities\n"

        self.econ_results_text.insert('1.0', output)

    def calculate_derating(self):
        """Calculate derating factors"""
        self.derating_text.delete('1.0', tk.END)
        output = "="*70 + "\n"
        output += "DERATING ANALYSIS\n"
        output += "="*70 + "\n\n"

        # Temperature derating
        ambient = self.ambient_temp_var.get()
        rated_ambient = 40  # Standard rating temperature

        if ambient > rated_ambient:
            temp_derating = 1 - (ambient - rated_ambient) * 0.015  # 1.5% per degree C
        else:
            temp_derating = 1.0

        output += "TEMPERATURE DERATING:\n"
        output += "-"*70 + "\n"
        output += f"Rated Ambient Temp:     {rated_ambient}°C\n"
        output += f"Actual Ambient Temp:    {ambient}°C\n"
        output += f"Temperature Derating:   {temp_derating:.3f} ({temp_derating*100:.1f}%)\n\n"

        # Altitude derating (assuming sea level for now)
        altitude = 0  # meters
        altitude_derating = 1 - max(0, (altitude - 1000) / 10000)

        output += "ALTITUDE DERATING:\n"
        output += "-"*70 + "\n"
        output += f"Altitude:               {altitude} m\n"
        output += f"Altitude Derating:      {altitude_derating:.3f} ({altitude_derating*100:.1f}%)\n\n"

        # Harmonic derating
        thd = 0.05  # 5% total harmonic distortion
        harmonic_derating = 1 / np.sqrt(1 + thd**2)

        output += "HARMONIC DERATING:\n"
        output += "-"*70 + "\n"
        output += f"THD:                    {thd*100:.1f}%\n"
        output += f"Harmonic Derating:      {harmonic_derating:.3f} ({harmonic_derating*100:.1f}%)\n\n"

        # Combined derating
        total_derating = temp_derating * altitude_derating * harmonic_derating

        output += "OVERALL DERATING:\n"
        output += "-"*70 + "\n"
        output += f"Total Derating Factor:  {total_derating:.3f} ({total_derating*100:.1f}%)\n\n"

        # Power consumption analysis
        if self.simulator.loss_history:
            avg_loss = np.mean([l['total'] for l in self.simulator.loss_history])
            rated_power = 100000  # 100 kW assumed rating
            derated_power = rated_power * total_derating

            output += "POWER CONSUMPTION ANALYSIS:\n"
            output += "-"*70 + "\n"
            output += f"Rated Power:            {rated_power/1000:.1f} kW\n"
            output += f"Derated Power:          {derated_power/1000:.1f} kW\n"
            output += f"Power Reduction:        {(rated_power - derated_power)/1000:.1f} kW\n"
            output += f"Average Losses:         {avg_loss/1000:.3f} kW\n"
            output += f"Efficiency at Derated:  {((derated_power-avg_loss)/derated_power)*100:.2f}%\n\n"

        output += "RECOMMENDATIONS:\n"
        output += "-"*70 + "\n"
        if temp_derating < 0.95:
            output += "• HIGH TEMPERATURE - Improve cooling or reduce load\n"
        if total_derating < 0.90:
            output += "• SIGNIFICANT DERATING - Review operating conditions\n"
        output += "• Monitor temperature continuously\n"
        output += "• Consider forced cooling if operating at high ambient\n"
        output += "• Regular cleaning of cooling surfaces recommended\n"

        self.derating_text.insert('1.0', output)

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        if self.auto_scale and hasattr(self, 'canvas'):
            try:
                self.fig.tight_layout()
                self.canvas.draw_idle()
            except:
                pass

    def save_results(self):
        """Save results to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"transformer_analysis_{timestamp}.txt"

        try:
            with open(filename, 'w') as f:
                f.write("PARALLEL TRANSFORMER ANALYSIS RESULTS\n")
                f.write("="*70 + "\n\n")
                f.write(self.results_text.get('1.0', tk.END))
                f.write("\n\nMULTI-PHYSICS SIMULATION RESULTS\n")
                f.write("="*70 + "\n\n")
                f.write(self.sim_results_text.get('1.0', tk.END))
                f.write("\n\nECONOMIC ANALYSIS RESULTS\n")
                f.write("="*70 + "\n\n")
                f.write(self.econ_results_text.get('1.0', tk.END))

            messagebox.showinfo("Success", f"Results saved to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save results: {str(e)}")

    def load_parameters(self):
        """Load parameters from file"""
        messagebox.showinfo("Info", "Load parameters feature - To be implemented")

    def show_about(self):
        """Show about dialog"""
        about_text = """
Advanced Parallel Transformer & Multi-Physics Simulation Lab
Version 1.0

Features:
• Parallel transformer load sharing analysis
• Multi-physics simulation (electromagnetic-thermal-mechanical)
• Real-time ODE solvers (RK45, Euler)
• Dynamic visualization
• Economic analysis with NPV calculations
• Advanced thermal and derating analysis
• Detailed loss breakdown (copper, iron, friction, stray)
• Auto-scaling interface

Developed for advanced electrical engineering education and research.
        """
        messagebox.showinfo("About", about_text)


def main():
    """Main entry point"""
    app = AdvancedTransformerLab()
    app.mainloop()


if __name__ == "__main__":
    main()
