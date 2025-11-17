"""
Advanced DC Motor Starter and Simulation System
Comprehensive tool for DC motor analysis, control, and multi-physics simulation
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from scipy.integrate import solve_ivp
import threading
import time
from datetime import datetime

class DCMotorStarterCalculator:
    """Calculates starter resistances for DC motors"""

    @staticmethod
    def calculate_problem_18(V=200, I_fl=12, Ra=0.3, Rf=100, I_max_factor=1.5, num_steps=5):
        """
        Problem 18: Calculate 5 steps in 6-stud starter

        Parameters:
        V: Supply voltage (V)
        I_fl: Full load current (A)
        Ra: Armature resistance (Ω)
        Rf: Field resistance (Ω)
        I_max_factor: Maximum current factor
        num_steps: Number of resistance steps

        Returns:
        List of resistance values for each step
        """
        # Field current
        If = V / Rf

        # Armature full-load current
        Ia_fl = I_fl - If

        # Maximum starting current
        I_max = I_max_factor * I_fl
        Ia_max = I_max - If

        # Minimum current (when to switch to next stud)
        Ia_min = Ia_fl

        # Current ratio
        alpha = Ia_max / Ia_min

        # Calculate total resistance at start
        R_total_start = V / Ia_max

        # Calculate resistance values for each step
        resistances = []

        # First step (total external resistance)
        R1_total = R_total_start - Ra
        resistances.append(R1_total)

        # Calculate subsequent steps using geometric progression
        for i in range(1, num_steps):
            R_next = resistances[i-1] / alpha
            resistances.append(R_next)

        # Calculate individual step resistances
        step_resistances = []
        for i in range(num_steps):
            if i == 0:
                step_resistances.append(resistances[0])
            else:
                step_resistances.append(resistances[i-1] - resistances[i])

        return step_resistances, {
            'If': If,
            'Ia_fl': Ia_fl,
            'Ia_max': Ia_max,
            'Ia_min': Ia_min,
            'alpha': alpha,
            'R_total_start': R_total_start
        }

    @staticmethod
    def calculate_problem_19(V=200, I_max_1=30, I_min_1=24, I_max_2=34, Ra=0.4):
        """
        Problem 19: Calculate resistance between first and second studs

        Parameters:
        V: Supply voltage (V)
        I_max_1: Maximum current on first stud (A)
        I_min_1: Minimum current on first stud (A)
        I_max_2: Maximum current on second stud (A)
        Ra: Armature resistance (Ω)

        Returns:
        Resistance between studs and calculation details
        """
        # Back EMF when current drops to 24A on first stud
        # At first stud: V = Eb + Ia * (Ra + R1)
        R1 = (V / I_max_1) - Ra

        # Back EMF at minimum current on first stud
        Eb = V - I_min_1 * (Ra + R1)

        # At second stud with same back EMF: V = Eb + Ia * (Ra + R2)
        R2 = (V - Eb) / I_max_2 - Ra

        # Resistance between first and second studs
        R_between = R1 - R2

        return R_between, {
            'R1': R1,
            'R2': R2,
            'Eb': Eb,
            'R_between': R_between
        }


class DCMotorPhysicsModel:
    """Multi-physics model for DC motor simulation"""

    def __init__(self, params):
        self.params = params
        self.update_derived_parameters()

    def update_derived_parameters(self):
        """Calculate derived parameters"""
        p = self.params

        # Electrical constants
        self.Ka = p['V_rated'] / (p['n_rated'] * 2 * np.pi / 60)  # Back EMF constant
        self.Kt = self.Ka  # Torque constant (in SI units, Ka = Kt)

        # Thermal capacitances (J/K)
        self.C_armature = p['mass_armature'] * 385  # Copper specific heat ~385 J/(kg·K)
        self.C_field = p['mass_field'] * 385
        self.C_housing = p['mass_housing'] * 450  # Steel specific heat ~450 J/(kg·K)

        # Thermal resistances (K/W)
        self.R_th_armature_housing = 1.0 / p['h_conv_armature']
        self.R_th_housing_ambient = 1.0 / p['h_conv_housing']

        # Moment of inertia
        self.J_total = p['J_rotor'] + p['J_load']

    def electrical_equations(self, t, state, R_external, T_load):
        """
        Electrical differential equations for DC motor

        State vector: [Ia, omega, theta, T_armature, T_field, T_housing]
        """
        Ia, omega, theta, T_arm, T_field, T_housing = state
        p = self.params

        # Back EMF
        Eb = self.Ka * omega

        # Applied voltage (can be time-varying)
        V_applied = p['V_rated']

        # Field current (assuming constant for shunt motor)
        If = p['V_rated'] / p['Rf']

        # Electrical equation: V = Eb + Ia*Ra + Ia*R_external
        dIa_dt = (V_applied - Eb - Ia * (p['Ra'] + R_external)) / p['La']

        # Torque developed
        T_em = self.Kt * If * Ia

        # Mechanical equation
        domega_dt = (T_em - T_load - p['B_friction'] * omega) / self.J_total

        # Angular position
        dtheta_dt = omega

        # === THERMAL MODEL ===

        # Copper losses (armature)
        P_copper_arm = Ia**2 * p['Ra']

        # Copper losses (field)
        P_copper_field = If**2 * p['Rf']

        # Iron losses (frequency dependent, approximate)
        f_magnetic = omega / (2 * np.pi)  # Frequency in Hz
        P_iron = p['P_iron_rated'] * (f_magnetic / (p['n_rated'] / 60))**1.5

        # Mechanical losses
        P_mechanical = p['B_friction'] * omega**2

        # Stray load losses (approximately 1% of output power)
        P_stray = 0.01 * abs(T_em * omega)

        # Heat generation in each component
        Q_gen_armature = P_copper_arm + 0.5 * P_iron + P_stray
        Q_gen_field = P_copper_field
        Q_gen_housing = P_mechanical + 0.5 * P_iron

        # Heat transfer between components
        Q_arm_to_housing = (T_arm - T_housing) / self.R_th_armature_housing
        Q_field_to_housing = (T_field - T_housing) / self.R_th_armature_housing
        Q_housing_to_ambient = (T_housing - p['T_ambient']) / self.R_th_housing_ambient

        # Temperature rate of change
        dT_arm_dt = (Q_gen_armature - Q_arm_to_housing) / self.C_armature
        dT_field_dt = (Q_gen_field - Q_field_to_housing) / self.C_field
        dT_housing_dt = (Q_gen_housing + Q_arm_to_housing + Q_field_to_housing - Q_housing_to_ambient) / self.C_housing

        # Store power losses for analysis
        self.last_losses = {
            'P_copper_arm': P_copper_arm,
            'P_copper_field': P_copper_field,
            'P_iron': P_iron,
            'P_mechanical': P_mechanical,
            'P_stray': P_stray,
            'P_total': P_copper_arm + P_copper_field + P_iron + P_mechanical + P_stray
        }

        return [dIa_dt, domega_dt, dtheta_dt, dT_arm_dt, dT_field_dt, dT_housing_dt]

    def calculate_mechanical_stress(self, Ia, omega):
        """Calculate mechanical stresses on shaft and bearings"""
        p = self.params

        # Electromagnetic torque
        If = p['V_rated'] / p['Rf']
        T_em = self.Kt * If * Ia

        # Shaft shear stress (τ = T*r/J_polar)
        r_shaft = p['shaft_diameter'] / 2
        J_polar = np.pi * r_shaft**4 / 2  # Polar moment of inertia
        tau_shaft = T_em * r_shaft / J_polar

        # Bearing radial load (simplified)
        # Assumes weight and belt tension
        F_radial = p['mass_rotor'] * 9.81 + abs(T_em) / (p['shaft_diameter'] / 2)

        # Bearing life calculation (L10 life in hours)
        # Using basic bearing life equation
        C_bearing = p['bearing_capacity']  # Dynamic load rating
        P_equiv = F_radial  # Equivalent dynamic load

        if P_equiv > 0:
            L10_revolutions = (C_bearing / P_equiv)**3 * 1e6
            n_rpm = omega * 60 / (2 * np.pi)
            L10_hours = L10_revolutions / (60 * n_rpm) if n_rpm > 0 else np.inf
        else:
            L10_hours = np.inf

        return {
            'T_em': T_em,
            'tau_shaft': tau_shaft,
            'F_radial': F_radial,
            'L10_hours': L10_hours,
            'safety_factor_shaft': p['tau_yield'] / (abs(tau_shaft) + 1e-9)
        }


class SimulationEngine:
    """Real-time simulation engine with multiple ODE solvers"""

    def __init__(self, motor_model):
        self.motor_model = motor_model
        self.solver_type = 'RK45'
        self.is_running = False
        self.results = {}

    def run_simulation(self, t_span, initial_state, R_external_func, T_load_func,
                      solver='RK45', max_step=0.001, callback=None):
        """
        Run motor simulation with specified solver

        Parameters:
        t_span: Tuple (t_start, t_end)
        initial_state: Initial state vector [Ia, omega, theta, T_arm, T_field, T_housing]
        R_external_func: Function R(t) for external resistance
        T_load_func: Function T_load(t) for load torque
        solver: 'RK45' or 'Euler'
        max_step: Maximum time step
        callback: Optional callback function for progress updates
        """

        def ode_func(t, y):
            R_ext = R_external_func(t)
            T_load = T_load_func(t)
            return self.motor_model.electrical_equations(t, y, R_ext, T_load)

        if solver == 'RK45':
            # Use scipy's RK45 solver
            sol = solve_ivp(
                ode_func,
                t_span,
                initial_state,
                method='RK45',
                max_step=max_step,
                dense_output=True,
                vectorized=False
            )

            # Extract results
            t = sol.t
            Ia = sol.y[0]
            omega = sol.y[1]
            theta = sol.y[2]
            T_arm = sol.y[3]
            T_field = sol.y[4]
            T_housing = sol.y[5]

        elif solver == 'Euler':
            # Implement explicit Euler method
            t0, tf = t_span
            dt = max_step
            t = np.arange(t0, tf, dt)
            n_steps = len(t)

            # Initialize state arrays
            Ia = np.zeros(n_steps)
            omega = np.zeros(n_steps)
            theta = np.zeros(n_steps)
            T_arm = np.zeros(n_steps)
            T_field = np.zeros(n_steps)
            T_housing = np.zeros(n_steps)

            # Set initial conditions
            y = np.array(initial_state)
            Ia[0], omega[0], theta[0], T_arm[0], T_field[0], T_housing[0] = y

            # Euler integration
            for i in range(1, n_steps):
                if callback and i % 100 == 0:
                    callback(i / n_steps)

                dydt = np.array(ode_func(t[i-1], y))
                y = y + dydt * dt

                Ia[i], omega[i], theta[i], T_arm[i], T_field[i], T_housing[i] = y

        else:
            raise ValueError(f"Unknown solver: {solver}")

        # Calculate derived quantities
        n_rpm = omega * 60 / (2 * np.pi)

        # Calculate back EMF
        Eb = self.motor_model.Ka * omega

        # Calculate electromagnetic torque
        If = self.motor_model.params['V_rated'] / self.motor_model.params['Rf']
        T_em = self.motor_model.Kt * If * Ia

        # Calculate power
        P_input = self.motor_model.params['V_rated'] * (Ia + If)
        P_output = T_em * omega
        efficiency = np.divide(P_output, P_input, out=np.zeros_like(P_output), where=P_input!=0) * 100

        self.results = {
            't': t,
            'Ia': Ia,
            'omega': omega,
            'n_rpm': n_rpm,
            'theta': theta,
            'T_arm': T_arm,
            'T_field': T_field,
            'T_housing': T_housing,
            'Eb': Eb,
            'T_em': T_em,
            'P_input': P_input,
            'P_output': P_output,
            'efficiency': efficiency
        }

        return self.results


class EconomicAnalyzer:
    """Economic analysis for motor operation"""

    @staticmethod
    def calculate_operating_cost(P_avg_kW, hours_per_year, electricity_cost_per_kWh,
                                efficiency_percent, power_factor=1.0):
        """Calculate annual operating cost"""

        # Annual energy consumption
        energy_kWh = P_avg_kW * hours_per_year / (efficiency_percent / 100)

        # Annual cost
        annual_cost = energy_kWh * electricity_cost_per_kWh

        return {
            'energy_kWh': energy_kWh,
            'annual_cost': annual_cost,
            'daily_cost': annual_cost / 365,
            'monthly_cost': annual_cost / 12
        }

    @staticmethod
    def lifecycle_cost_analysis(initial_cost, annual_operating_cost, maintenance_cost,
                               lifetime_years, discount_rate):
        """Calculate lifecycle cost with NPV"""

        total_cost = initial_cost
        npv_operating = 0
        npv_maintenance = 0

        for year in range(1, lifetime_years + 1):
            discount_factor = 1 / (1 + discount_rate)**year
            npv_operating += annual_operating_cost * discount_factor
            npv_maintenance += maintenance_cost * discount_factor

        total_npv = initial_cost + npv_operating + npv_maintenance

        return {
            'initial_cost': initial_cost,
            'npv_operating': npv_operating,
            'npv_maintenance': npv_maintenance,
            'total_npv': total_npv,
            'annual_equivalent': total_npv * discount_rate / (1 - (1 + discount_rate)**(-lifetime_years))
        }


class AdvancedDCMotorGUI:
    """Main GUI application for DC Motor simulation and analysis"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Starter & Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Simulation state
        self.is_simulating = False
        self.simulation_thread = None

        # Default motor parameters
        self.default_params = {
            # Electrical parameters
            'V_rated': 200.0,
            'I_rated': 12.0,
            'Ra': 0.3,
            'La': 0.05,  # Armature inductance (H)
            'Rf': 100.0,
            'n_rated': 1500.0,  # Rated speed (RPM)
            'P_rated': 2000.0,  # Rated power (W)

            # Thermal parameters
            'T_ambient': 25.0,  # Ambient temperature (°C)
            'T_max_armature': 155.0,  # Maximum armature temperature (°C)
            'mass_armature': 5.0,  # kg
            'mass_field': 3.0,  # kg
            'mass_housing': 10.0,  # kg
            'h_conv_armature': 50.0,  # Heat transfer coefficient (W/K)
            'h_conv_housing': 20.0,  # Heat transfer coefficient (W/K)
            'P_iron_rated': 50.0,  # Iron losses at rated speed (W)

            # Mechanical parameters
            'J_rotor': 0.05,  # Rotor inertia (kg·m²)
            'J_load': 0.02,  # Load inertia (kg·m²)
            'B_friction': 0.01,  # Friction coefficient (N·m·s)
            'shaft_diameter': 0.025,  # Shaft diameter (m)
            'tau_yield': 250e6,  # Shaft yield strength (Pa)
            'mass_rotor': 8.0,  # Rotor mass (kg)
            'bearing_capacity': 5000.0,  # Bearing dynamic load rating (N)

            # Economic parameters
            'electricity_cost': 0.12,  # $/kWh
            'operating_hours': 4000,  # hours/year
            'initial_cost': 1500.0,  # $
            'maintenance_cost': 200.0,  # $/year
            'lifetime_years': 15,
            'discount_rate': 0.05
        }

        self.params = self.default_params.copy()

        # Configure grid weight for auto-scaling
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Create main notebook
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.create_starter_calc_tab()
        self.create_simulation_tab()
        self.create_thermal_tab()
        self.create_mechanical_tab()
        self.create_economic_tab()
        self.create_control_tab()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # This ensures plots and widgets scale with window
        pass

    def create_starter_calc_tab(self):
        """Tab for starter resistance calculations"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Starter Calculator")

        # Configure grid weights
        tab.grid_rowconfigure(2, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Problem 18 Frame
        frame18 = ttk.LabelFrame(tab, text="Problem 18: 6-Stud Starter Design", padding=10)
        frame18.grid(row=0, column=0, sticky='ew', padx=10, pady=5)

        # Input fields for Problem 18
        inputs18 = ttk.Frame(frame18)
        inputs18.pack(fill='x')

        ttk.Label(inputs18, text="Voltage (V):").grid(row=0, column=0, sticky='w', padx=5)
        self.p18_V = ttk.Entry(inputs18, width=10)
        self.p18_V.insert(0, "200")
        self.p18_V.grid(row=0, column=1, padx=5)

        ttk.Label(inputs18, text="Full-load Current (A):").grid(row=0, column=2, sticky='w', padx=5)
        self.p18_Ifl = ttk.Entry(inputs18, width=10)
        self.p18_Ifl.insert(0, "12")
        self.p18_Ifl.grid(row=0, column=3, padx=5)

        ttk.Label(inputs18, text="Ra (Ω):").grid(row=1, column=0, sticky='w', padx=5)
        self.p18_Ra = ttk.Entry(inputs18, width=10)
        self.p18_Ra.insert(0, "0.3")
        self.p18_Ra.grid(row=1, column=1, padx=5)

        ttk.Label(inputs18, text="Rf (Ω):").grid(row=1, column=2, sticky='w', padx=5)
        self.p18_Rf = ttk.Entry(inputs18, width=10)
        self.p18_Rf.insert(0, "100")
        self.p18_Rf.grid(row=1, column=3, padx=5)

        ttk.Label(inputs18, text="Max Current Factor:").grid(row=2, column=0, sticky='w', padx=5)
        self.p18_factor = ttk.Entry(inputs18, width=10)
        self.p18_factor.insert(0, "1.5")
        self.p18_factor.grid(row=2, column=1, padx=5)

        ttk.Button(inputs18, text="Calculate", command=self.calculate_problem18).grid(row=2, column=3, padx=5, pady=5)

        # Results for Problem 18
        self.result18_text = scrolledtext.ScrolledText(frame18, height=10, width=80)
        self.result18_text.pack(fill='both', expand=True, pady=5)

        # Problem 19 Frame
        frame19 = ttk.LabelFrame(tab, text="Problem 19: Resistance Between Studs", padding=10)
        frame19.grid(row=1, column=0, sticky='ew', padx=10, pady=5)

        # Input fields for Problem 19
        inputs19 = ttk.Frame(frame19)
        inputs19.pack(fill='x')

        ttk.Label(inputs19, text="Voltage (V):").grid(row=0, column=0, sticky='w', padx=5)
        self.p19_V = ttk.Entry(inputs19, width=10)
        self.p19_V.insert(0, "200")
        self.p19_V.grid(row=0, column=1, padx=5)

        ttk.Label(inputs19, text="I_max_1 (A):").grid(row=0, column=2, sticky='w', padx=5)
        self.p19_Imax1 = ttk.Entry(inputs19, width=10)
        self.p19_Imax1.insert(0, "30")
        self.p19_Imax1.grid(row=0, column=3, padx=5)

        ttk.Label(inputs19, text="I_min_1 (A):").grid(row=1, column=0, sticky='w', padx=5)
        self.p19_Imin1 = ttk.Entry(inputs19, width=10)
        self.p19_Imin1.insert(0, "24")
        self.p19_Imin1.grid(row=1, column=1, padx=5)

        ttk.Label(inputs19, text="I_max_2 (A):").grid(row=1, column=2, sticky='w', padx=5)
        self.p19_Imax2 = ttk.Entry(inputs19, width=10)
        self.p19_Imax2.insert(0, "34")
        self.p19_Imax2.grid(row=1, column=3, padx=5)

        ttk.Label(inputs19, text="Ra (Ω):").grid(row=2, column=0, sticky='w', padx=5)
        self.p19_Ra = ttk.Entry(inputs19, width=10)
        self.p19_Ra.insert(0, "0.4")
        self.p19_Ra.grid(row=2, column=1, padx=5)

        ttk.Button(inputs19, text="Calculate", command=self.calculate_problem19).grid(row=2, column=3, padx=5, pady=5)

        # Results for Problem 19
        self.result19_text = scrolledtext.ScrolledText(frame19, height=10, width=80)
        self.result19_text.pack(fill='both', expand=True, pady=5)

    def create_simulation_tab(self):
        """Tab for dynamic motor simulation"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        # Configure grid weights
        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=10, pady=5)

        # Parameter inputs
        param_frame = ttk.Frame(control_frame)
        param_frame.pack(fill='x')

        ttk.Label(param_frame, text="Voltage (V):").grid(row=0, column=0, sticky='w')
        self.sim_voltage = ttk.Scale(param_frame, from_=0, to=250, orient='horizontal', length=200,
                                     command=self.update_voltage_label)
        self.sim_voltage.set(200)
        self.sim_voltage.grid(row=0, column=1, padx=5)
        self.voltage_label = ttk.Label(param_frame, text="200 V")
        self.voltage_label.grid(row=0, column=2)

        ttk.Label(param_frame, text="Load Torque (N·m):").grid(row=1, column=0, sticky='w')
        self.sim_torque = ttk.Scale(param_frame, from_=0, to=50, orient='horizontal', length=200,
                                    command=self.update_torque_label)
        self.sim_torque.set(10)
        self.sim_torque.grid(row=1, column=1, padx=5)
        self.torque_label = ttk.Label(param_frame, text="10 N·m")
        self.torque_label.grid(row=1, column=2)

        ttk.Label(param_frame, text="External R (Ω):").grid(row=2, column=0, sticky='w')
        self.sim_resistance = ttk.Scale(param_frame, from_=0, to=10, orient='horizontal', length=200,
                                       command=self.update_resistance_label)
        self.sim_resistance.set(0)
        self.sim_resistance.grid(row=2, column=1, padx=5)
        self.resistance_label = ttk.Label(param_frame, text="0 Ω")
        self.resistance_label.grid(row=2, column=2)

        ttk.Label(param_frame, text="Simulation Time (s):").grid(row=3, column=0, sticky='w')
        self.sim_time = ttk.Entry(param_frame, width=10)
        self.sim_time.insert(0, "5.0")
        self.sim_time.grid(row=3, column=1, sticky='w', padx=5)

        ttk.Label(param_frame, text="Solver:").grid(row=4, column=0, sticky='w')
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(param_frame, text="RK45", variable=self.solver_var, value='RK45').grid(row=4, column=1, sticky='w')
        ttk.Radiobutton(param_frame, text="Euler", variable=self.solver_var, value='Euler').grid(row=4, column=2, sticky='w')

        # Buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill='x', pady=5)

        self.start_btn = ttk.Button(button_frame, text="Start", command=self.start_simulation)
        self.start_btn.pack(side='left', padx=5)

        self.stop_btn = ttk.Button(button_frame, text="Stop", command=self.stop_simulation, state='disabled')
        self.stop_btn.pack(side='left', padx=5)

        ttk.Button(button_frame, text="Reset", command=self.reset_simulation).pack(side='left', padx=5)

        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(button_frame, variable=self.progress_var, maximum=100, length=200)
        self.progress_bar.pack(side='left', padx=10)

        # Plots
        plot_frame = ttk.Frame(tab)
        plot_frame.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_rowconfigure(1, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(1, weight=1)

        # Create figure with subplots
        self.sim_fig = Figure(figsize=(12, 8), dpi=100)
        self.sim_axes = []

        self.sim_axes.append(self.sim_fig.add_subplot(2, 2, 1))
        self.sim_axes[0].set_title('Armature Current vs Time')
        self.sim_axes[0].set_xlabel('Time (s)')
        self.sim_axes[0].set_ylabel('Current (A)')
        self.sim_axes[0].grid(True)

        self.sim_axes.append(self.sim_fig.add_subplot(2, 2, 2))
        self.sim_axes[1].set_title('Speed vs Time')
        self.sim_axes[1].set_xlabel('Time (s)')
        self.sim_axes[1].set_ylabel('Speed (RPM)')
        self.sim_axes[1].grid(True)

        self.sim_axes.append(self.sim_fig.add_subplot(2, 2, 3))
        self.sim_axes[2].set_title('Torque vs Time')
        self.sim_axes[2].set_xlabel('Time (s)')
        self.sim_axes[2].set_ylabel('Torque (N·m)')
        self.sim_axes[2].grid(True)

        self.sim_axes.append(self.sim_fig.add_subplot(2, 2, 4))
        self.sim_axes[3].set_title('Efficiency vs Time')
        self.sim_axes[3].set_xlabel('Time (s)')
        self.sim_axes[3].set_ylabel('Efficiency (%)')
        self.sim_axes[3].grid(True)

        self.sim_fig.tight_layout()

        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, plot_frame)
        self.sim_canvas.get_tk_widget().grid(row=0, column=0, columnspan=2, sticky='nsew')

        # Navigation toolbar
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.grid(row=1, column=0, columnspan=2, sticky='ew')
        toolbar = NavigationToolbar2Tk(self.sim_canvas, toolbar_frame)
        toolbar.update()

    def create_thermal_tab(self):
        """Tab for thermal analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Thermal Analysis")

        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Controls
        control_frame = ttk.LabelFrame(tab, text="Thermal Parameters", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=10, pady=5)

        ttk.Label(control_frame, text="Ambient Temp (°C):").grid(row=0, column=0, sticky='w')
        self.thermal_ambient = ttk.Entry(control_frame, width=10)
        self.thermal_ambient.insert(0, "25")
        self.thermal_ambient.grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="Max Operating Temp (°C):").grid(row=0, column=2, sticky='w')
        self.thermal_max = ttk.Entry(control_frame, width=10)
        self.thermal_max.insert(0, "155")
        self.thermal_max.grid(row=0, column=3, padx=5)

        ttk.Button(control_frame, text="Run Thermal Simulation",
                  command=self.run_thermal_simulation).grid(row=0, column=4, padx=10)

        # Plots
        plot_frame = ttk.Frame(tab)
        plot_frame.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)

        self.thermal_fig = Figure(figsize=(12, 8), dpi=100)

        self.thermal_ax1 = self.thermal_fig.add_subplot(2, 2, 1)
        self.thermal_ax1.set_title('Temperature vs Time')
        self.thermal_ax1.set_xlabel('Time (s)')
        self.thermal_ax1.set_ylabel('Temperature (°C)')
        self.thermal_ax1.grid(True)

        self.thermal_ax2 = self.thermal_fig.add_subplot(2, 2, 2)
        self.thermal_ax2.set_title('Power Losses Breakdown')
        self.thermal_ax2.set_ylabel('Power Loss (W)')

        self.thermal_ax3 = self.thermal_fig.add_subplot(2, 2, 3)
        self.thermal_ax3.set_title('Heat Flow Diagram')
        self.thermal_ax3.set_xlabel('Time (s)')
        self.thermal_ax3.set_ylabel('Heat Flow (W)')
        self.thermal_ax3.grid(True)

        self.thermal_ax4 = self.thermal_fig.add_subplot(2, 2, 4)
        self.thermal_ax4.set_title('Derating Curve')
        self.thermal_ax4.set_xlabel('Ambient Temperature (°C)')
        self.thermal_ax4.set_ylabel('Power Capacity (%)')
        self.thermal_ax4.grid(True)

        self.thermal_fig.tight_layout()

        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, plot_frame)
        self.thermal_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_mechanical_tab(self):
        """Tab for mechanical stress analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Mechanical Analysis")

        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Controls
        control_frame = ttk.LabelFrame(tab, text="Mechanical Parameters", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=10, pady=5)

        ttk.Label(control_frame, text="Shaft Diameter (mm):").grid(row=0, column=0, sticky='w')
        self.mech_shaft_dia = ttk.Entry(control_frame, width=10)
        self.mech_shaft_dia.insert(0, "25")
        self.mech_shaft_dia.grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="Bearing Capacity (N):").grid(row=0, column=2, sticky='w')
        self.mech_bearing = ttk.Entry(control_frame, width=10)
        self.mech_bearing.insert(0, "5000")
        self.mech_bearing.grid(row=0, column=3, padx=5)

        ttk.Button(control_frame, text="Analyze Mechanical Stress",
                  command=self.analyze_mechanical_stress).grid(row=0, column=4, padx=10)

        # Results display
        result_frame = ttk.Frame(tab)
        result_frame.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)
        result_frame.grid_rowconfigure(0, weight=1)
        result_frame.grid_columnconfigure(0, weight=1)

        self.mech_fig = Figure(figsize=(12, 8), dpi=100)

        self.mech_ax1 = self.mech_fig.add_subplot(2, 2, 1)
        self.mech_ax1.set_title('Shaft Shear Stress vs Torque')
        self.mech_ax1.set_xlabel('Torque (N·m)')
        self.mech_ax1.set_ylabel('Shear Stress (MPa)')
        self.mech_ax1.grid(True)

        self.mech_ax2 = self.mech_fig.add_subplot(2, 2, 2)
        self.mech_ax2.set_title('Bearing Load vs Speed')
        self.mech_ax2.set_xlabel('Speed (RPM)')
        self.mech_ax2.set_ylabel('Radial Load (N)')
        self.mech_ax2.grid(True)

        self.mech_ax3 = self.mech_fig.add_subplot(2, 2, 3)
        self.mech_ax3.set_title('Bearing Life (L10) vs Load')
        self.mech_ax3.set_xlabel('Load (N)')
        self.mech_ax3.set_ylabel('L10 Life (hours)')
        self.mech_ax3.set_yscale('log')
        self.mech_ax3.grid(True)

        self.mech_ax4 = self.mech_fig.add_subplot(2, 2, 4)
        self.mech_ax4.set_title('Safety Factors')

        self.mech_fig.tight_layout()

        self.mech_canvas = FigureCanvasTkAgg(self.mech_fig, result_frame)
        self.mech_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_economic_tab(self):
        """Tab for economic analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Economic Analysis")

        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Input frame
        input_frame = ttk.LabelFrame(tab, text="Economic Parameters", padding=10)
        input_frame.grid(row=0, column=0, sticky='ew', padx=10, pady=5)

        ttk.Label(input_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w')
        self.econ_elec_cost = ttk.Entry(input_frame, width=10)
        self.econ_elec_cost.insert(0, "0.12")
        self.econ_elec_cost.grid(row=0, column=1, padx=5)

        ttk.Label(input_frame, text="Operating Hours/Year:").grid(row=0, column=2, sticky='w')
        self.econ_hours = ttk.Entry(input_frame, width=10)
        self.econ_hours.insert(0, "4000")
        self.econ_hours.grid(row=0, column=3, padx=5)

        ttk.Label(input_frame, text="Average Power (kW):").grid(row=1, column=0, sticky='w')
        self.econ_power = ttk.Entry(input_frame, width=10)
        self.econ_power.insert(0, "2.0")
        self.econ_power.grid(row=1, column=1, padx=5)

        ttk.Label(input_frame, text="Efficiency (%):").grid(row=1, column=2, sticky='w')
        self.econ_efficiency = ttk.Entry(input_frame, width=10)
        self.econ_efficiency.insert(0, "85")
        self.econ_efficiency.grid(row=1, column=3, padx=5)

        ttk.Label(input_frame, text="Initial Cost ($):").grid(row=2, column=0, sticky='w')
        self.econ_initial = ttk.Entry(input_frame, width=10)
        self.econ_initial.insert(0, "1500")
        self.econ_initial.grid(row=2, column=1, padx=5)

        ttk.Label(input_frame, text="Annual Maintenance ($):").grid(row=2, column=2, sticky='w')
        self.econ_maint = ttk.Entry(input_frame, width=10)
        self.econ_maint.insert(0, "200")
        self.econ_maint.grid(row=2, column=3, padx=5)

        ttk.Label(input_frame, text="Lifetime (years):").grid(row=3, column=0, sticky='w')
        self.econ_lifetime = ttk.Entry(input_frame, width=10)
        self.econ_lifetime.insert(0, "15")
        self.econ_lifetime.grid(row=3, column=1, padx=5)

        ttk.Label(input_frame, text="Discount Rate (%):").grid(row=3, column=2, sticky='w')
        self.econ_discount = ttk.Entry(input_frame, width=10)
        self.econ_discount.insert(0, "5")
        self.econ_discount.grid(row=3, column=3, padx=5)

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=4, column=0, columnspan=4, pady=10)

        # Results frame
        result_frame = ttk.Frame(tab)
        result_frame.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)
        result_frame.grid_rowconfigure(0, weight=1)
        result_frame.grid_columnconfigure(0, weight=1)
        result_frame.grid_columnconfigure(1, weight=1)

        # Text results
        text_frame = ttk.LabelFrame(result_frame, text="Cost Analysis Results", padding=10)
        text_frame.grid(row=0, column=0, sticky='nsew', padx=5)

        self.econ_results_text = scrolledtext.ScrolledText(text_frame, height=20, width=50)
        self.econ_results_text.pack(fill='both', expand=True)

        # Plots
        plot_frame = ttk.Frame(result_frame)
        plot_frame.grid(row=0, column=1, sticky='nsew', padx=5)
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)

        self.econ_fig = Figure(figsize=(6, 8), dpi=100)

        self.econ_ax1 = self.econ_fig.add_subplot(2, 1, 1)
        self.econ_ax1.set_title('Cost Breakdown')

        self.econ_ax2 = self.econ_fig.add_subplot(2, 1, 2)
        self.econ_ax2.set_title('Cumulative Cost vs Time')
        self.econ_ax2.set_xlabel('Year')
        self.econ_ax2.set_ylabel('Cumulative Cost ($)')
        self.econ_ax2.grid(True)

        self.econ_fig.tight_layout()

        self.econ_canvas = FigureCanvasTkAgg(self.econ_fig, plot_frame)
        self.econ_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_control_tab(self):
        """Tab for advanced control methods"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Advanced Controls")

        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Control method selection
        control_frame = ttk.LabelFrame(tab, text="Control Methods", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=10, pady=5)

        self.control_method = tk.StringVar(value='open_loop')

        ttk.Radiobutton(control_frame, text="Open Loop (Direct Start)",
                       variable=self.control_method, value='open_loop').grid(row=0, column=0, sticky='w', padx=5)
        ttk.Radiobutton(control_frame, text="Stepped Resistance Starter",
                       variable=self.control_method, value='stepped').grid(row=0, column=1, sticky='w', padx=5)
        ttk.Radiobutton(control_frame, text="Soft Start (Voltage Ramp)",
                       variable=self.control_method, value='soft_start').grid(row=0, column=2, sticky='w', padx=5)
        ttk.Radiobutton(control_frame, text="PID Speed Control",
                       variable=self.control_method, value='pid').grid(row=0, column=3, sticky='w', padx=5)

        # PID parameters
        pid_frame = ttk.LabelFrame(tab, text="PID Parameters", padding=10)
        pid_frame.grid(row=1, column=0, sticky='ew', padx=10, pady=5)

        ttk.Label(pid_frame, text="Kp:").grid(row=0, column=0, sticky='w')
        self.pid_kp = ttk.Entry(pid_frame, width=10)
        self.pid_kp.insert(0, "0.1")
        self.pid_kp.grid(row=0, column=1, padx=5)

        ttk.Label(pid_frame, text="Ki:").grid(row=0, column=2, sticky='w')
        self.pid_ki = ttk.Entry(pid_frame, width=10)
        self.pid_ki.insert(0, "0.01")
        self.pid_ki.grid(row=0, column=3, padx=5)

        ttk.Label(pid_frame, text="Kd:").grid(row=0, column=4, sticky='w')
        self.pid_kd = ttk.Entry(pid_frame, width=10)
        self.pid_kd.insert(0, "0.001")
        self.pid_kd.grid(row=0, column=5, padx=5)

        ttk.Label(pid_frame, text="Setpoint (RPM):").grid(row=1, column=0, sticky='w')
        self.pid_setpoint = ttk.Entry(pid_frame, width=10)
        self.pid_setpoint.insert(0, "1500")
        self.pid_setpoint.grid(row=1, column=1, padx=5)

        ttk.Button(pid_frame, text="Apply Control",
                  command=self.apply_control).grid(row=1, column=5, padx=5)

        # Control info
        info_frame = ttk.Frame(tab)
        info_frame.grid(row=2, column=0, sticky='nsew', padx=10, pady=5)
        info_frame.grid_rowconfigure(0, weight=1)
        info_frame.grid_columnconfigure(0, weight=1)

        self.control_info = scrolledtext.ScrolledText(info_frame, height=15)
        self.control_info.grid(row=0, column=0, sticky='nsew')

        # Add control method descriptions
        control_desc = """
CONTROL METHOD DESCRIPTIONS:

1. Open Loop (Direct Start):
   - Motor is directly connected to supply voltage
   - Highest starting current (5-8x rated)
   - Fastest acceleration
   - Maximum stress on motor and mechanical system

2. Stepped Resistance Starter:
   - External resistances are progressively cut out
   - Limits starting current to 1.5-2.5x rated
   - Stepped acceleration profile
   - Energy dissipated in external resistances

3. Soft Start (Voltage Ramp):
   - Voltage is gradually increased from 0 to rated
   - Smooth current and torque profiles
   - Reduced mechanical stress
   - Requires power electronic converter

4. PID Speed Control:
   - Closed-loop control maintains desired speed
   - Compensates for load variations
   - Requires speed sensor (encoder/tachometer)
   - Tuning parameters: Kp (proportional), Ki (integral), Kd (derivative)
        """
        self.control_info.insert('1.0', control_desc)
        self.control_info.config(state='disabled')

    # Callback functions for sliders
    def update_voltage_label(self, value):
        self.voltage_label.config(text=f"{float(value):.1f} V")
        self.params['V_rated'] = float(value)

    def update_torque_label(self, value):
        self.torque_label.config(text=f"{float(value):.1f} N·m")

    def update_resistance_label(self, value):
        self.resistance_label.config(text=f"{float(value):.2f} Ω")

    # Calculation methods
    def calculate_problem18(self):
        """Calculate and display Problem 18 results"""
        try:
            V = float(self.p18_V.get())
            I_fl = float(self.p18_Ifl.get())
            Ra = float(self.p18_Ra.get())
            Rf = float(self.p18_Rf.get())
            factor = float(self.p18_factor.get())

            resistances, details = DCMotorStarterCalculator.calculate_problem_18(
                V, I_fl, Ra, Rf, factor, 5
            )

            # Display results
            result_text = "=" * 70 + "\n"
            result_text += "PROBLEM 18: 6-STUD STARTER RESISTANCE CALCULATION\n"
            result_text += "=" * 70 + "\n\n"

            result_text += "INPUT PARAMETERS:\n"
            result_text += f"  Supply Voltage (V)         : {V} V\n"
            result_text += f"  Full-load Current (I_fl)   : {I_fl} A\n"
            result_text += f"  Armature Resistance (Ra)   : {Ra} Ω\n"
            result_text += f"  Field Resistance (Rf)      : {Rf} Ω\n"
            result_text += f"  Max Current Factor         : {factor}\n\n"

            result_text += "CALCULATED VALUES:\n"
            result_text += f"  Field Current (If)         : {details['If']:.3f} A\n"
            result_text += f"  Armature Current (Ia_fl)   : {details['Ia_fl']:.3f} A\n"
            result_text += f"  Maximum Starting Current   : {details['Ia_max']:.3f} A\n"
            result_text += f"  Current Ratio (α)          : {details['alpha']:.4f}\n"
            result_text += f"  Total Start Resistance     : {details['R_total_start']:.3f} Ω\n\n"

            result_text += "STARTER RESISTANCE VALUES:\n"
            result_text += "-" * 50 + "\n"
            result_text += f"{'Step':<8} {'Resistance (Ω)':<20} {'Expected':<15}\n"
            result_text += "-" * 50 + "\n"

            expected = [6.57, 3.12, 1.48, 0.7, 0.33]
            for i, (R, exp) in enumerate(zip(resistances, expected)):
                result_text += f"{i+1:<8} {R:<20.2f} {exp:<15.2f}\n"

            result_text += "\n" + "=" * 70 + "\n"
            result_text += "VERIFICATION:\n"
            result_text += f"Expected values: {expected}\n"
            result_text += f"Calculated values: {[round(r, 2) for r in resistances]}\n"
            result_text += "=" * 70 + "\n"

            self.result18_text.delete('1.0', tk.END)
            self.result18_text.insert('1.0', result_text)

        except Exception as e:
            messagebox.showerror("Error", f"Calculation error: {str(e)}")

    def calculate_problem19(self):
        """Calculate and display Problem 19 results"""
        try:
            V = float(self.p19_V.get())
            I_max_1 = float(self.p19_Imax1.get())
            I_min_1 = float(self.p19_Imin1.get())
            I_max_2 = float(self.p19_Imax2.get())
            Ra = float(self.p19_Ra.get())

            R_between, details = DCMotorStarterCalculator.calculate_problem_19(
                V, I_max_1, I_min_1, I_max_2, Ra
            )

            # Display results
            result_text = "=" * 70 + "\n"
            result_text += "PROBLEM 19: RESISTANCE BETWEEN STARTER STUDS\n"
            result_text += "=" * 70 + "\n\n"

            result_text += "INPUT PARAMETERS:\n"
            result_text += f"  Supply Voltage (V)              : {V} V\n"
            result_text += f"  Max Current on 1st Stud (I1max) : {I_max_1} A\n"
            result_text += f"  Min Current on 1st Stud (I1min) : {I_min_1} A\n"
            result_text += f"  Max Current on 2nd Stud (I2max) : {I_max_2} A\n"
            result_text += f"  Armature Resistance (Ra)        : {Ra} Ω\n\n"

            result_text += "CALCULATION STEPS:\n"
            result_text += "-" * 70 + "\n"
            result_text += f"1. Total resistance at 1st stud:\n"
            result_text += f"   R1 = V/I_max_1 - Ra = {V}/{I_max_1} - {Ra}\n"
            result_text += f"   R1 = {details['R1']:.4f} Ω\n\n"

            result_text += f"2. Back EMF when current drops to {I_min_1} A:\n"
            result_text += f"   Eb = V - I_min_1 * (Ra + R1)\n"
            result_text += f"   Eb = {V} - {I_min_1} * ({Ra} + {details['R1']:.4f})\n"
            result_text += f"   Eb = {details['Eb']:.4f} V\n\n"

            result_text += f"3. Total resistance at 2nd stud:\n"
            result_text += f"   R2 = (V - Eb)/I_max_2 - Ra\n"
            result_text += f"   R2 = ({V} - {details['Eb']:.4f})/{I_max_2} - {Ra}\n"
            result_text += f"   R2 = {details['R2']:.4f} Ω\n\n"

            result_text += f"4. Resistance between 1st and 2nd studs:\n"
            result_text += f"   R_between = R1 - R2\n"
            result_text += f"   R_between = {details['R1']:.4f} - {details['R2']:.4f}\n"
            result_text += f"   R_between = {R_between:.4f} Ω\n\n"

            result_text += "=" * 70 + "\n"
            result_text += "FINAL RESULT:\n"
            result_text += f"  Resistance between studs = {R_between:.4f} Ω\n"
            result_text += "=" * 70 + "\n"

            self.result19_text.delete('1.0', tk.END)
            self.result19_text.insert('1.0', result_text)

        except Exception as e:
            messagebox.showerror("Error", f"Calculation error: {str(e)}")

    def start_simulation(self):
        """Start dynamic motor simulation"""
        if self.is_simulating:
            messagebox.showwarning("Warning", "Simulation already running!")
            return

        self.is_simulating = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')

        # Run simulation in separate thread
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

    def stop_simulation(self):
        """Stop simulation"""
        self.is_simulating = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()

        # Clear plots
        for ax in self.sim_axes:
            ax.clear()

        self.sim_axes[0].set_title('Armature Current vs Time')
        self.sim_axes[0].set_xlabel('Time (s)')
        self.sim_axes[0].set_ylabel('Current (A)')
        self.sim_axes[0].grid(True)

        self.sim_axes[1].set_title('Speed vs Time')
        self.sim_axes[1].set_xlabel('Time (s)')
        self.sim_axes[1].set_ylabel('Speed (RPM)')
        self.sim_axes[1].grid(True)

        self.sim_axes[2].set_title('Torque vs Time')
        self.sim_axes[2].set_xlabel('Time (s)')
        self.sim_axes[2].set_ylabel('Torque (N·m)')
        self.sim_axes[2].grid(True)

        self.sim_axes[3].set_title('Efficiency vs Time')
        self.sim_axes[3].set_xlabel('Time (s)')
        self.sim_axes[3].set_ylabel('Efficiency (%)')
        self.sim_axes[3].grid(True)

        self.sim_canvas.draw()
        self.progress_var.set(0)

    def run_simulation(self):
        """Run the motor simulation"""
        try:
            # Get parameters
            t_end = float(self.sim_time.get())
            T_load = float(self.sim_torque.get())
            R_ext = float(self.sim_resistance.get())
            solver = self.solver_var.get()

            # Create motor model
            motor = DCMotorPhysicsModel(self.params)
            sim_engine = SimulationEngine(motor)

            # Initial conditions: [Ia, omega, theta, T_arm, T_field, T_housing]
            initial_state = [0.0, 0.0, 0.0,
                           self.params['T_ambient'],
                           self.params['T_ambient'],
                           self.params['T_ambient']]

            # Define external resistance and load torque as functions of time
            def R_external_func(t):
                return R_ext

            def T_load_func(t):
                # Step load at t=1s
                return T_load if t > 1.0 else 0.0

            # Progress callback
            def progress_callback(progress):
                self.progress_var.set(progress * 100)

            # Run simulation
            results = sim_engine.run_simulation(
                (0, t_end),
                initial_state,
                R_external_func,
                T_load_func,
                solver=solver,
                max_step=0.001,
                callback=progress_callback
            )

            # Update plots
            self.root.after(0, self.update_simulation_plots, results)

            self.progress_var.set(100)

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation: {str(e)}")
        finally:
            self.is_simulating = False
            self.root.after(0, lambda: self.start_btn.config(state='normal'))
            self.root.after(0, lambda: self.stop_btn.config(state='disabled'))

    def update_simulation_plots(self, results):
        """Update simulation plots with results"""
        # Clear previous plots
        for ax in self.sim_axes:
            ax.clear()

        # Plot current
        self.sim_axes[0].plot(results['t'], results['Ia'], 'b-', linewidth=2)
        self.sim_axes[0].set_title('Armature Current vs Time')
        self.sim_axes[0].set_xlabel('Time (s)')
        self.sim_axes[0].set_ylabel('Current (A)')
        self.sim_axes[0].grid(True)

        # Plot speed
        self.sim_axes[1].plot(results['t'], results['n_rpm'], 'r-', linewidth=2)
        self.sim_axes[1].set_title('Speed vs Time')
        self.sim_axes[1].set_xlabel('Time (s)')
        self.sim_axes[1].set_ylabel('Speed (RPM)')
        self.sim_axes[1].grid(True)

        # Plot torque
        self.sim_axes[2].plot(results['t'], results['T_em'], 'g-', linewidth=2)
        self.sim_axes[2].set_title('Electromagnetic Torque vs Time')
        self.sim_axes[2].set_xlabel('Time (s)')
        self.sim_axes[2].set_ylabel('Torque (N·m)')
        self.sim_axes[2].grid(True)

        # Plot efficiency
        self.sim_axes[3].plot(results['t'], results['efficiency'], 'm-', linewidth=2)
        self.sim_axes[3].set_title('Efficiency vs Time')
        self.sim_axes[3].set_xlabel('Time (s)')
        self.sim_axes[3].set_ylabel('Efficiency (%)')
        self.sim_axes[3].grid(True)

        self.sim_fig.tight_layout()
        self.sim_canvas.draw()

    def run_thermal_simulation(self):
        """Run thermal analysis simulation"""
        try:
            # Update thermal parameters
            self.params['T_ambient'] = float(self.thermal_ambient.get())
            self.params['T_max_armature'] = float(self.thermal_max.get())

            # Create motor model and run simulation
            motor = DCMotorPhysicsModel(self.params)
            sim_engine = SimulationEngine(motor)

            # Run longer simulation for thermal analysis
            initial_state = [10.0, 0.0, 0.0,  # Start with 10A current
                           self.params['T_ambient'],
                           self.params['T_ambient'],
                           self.params['T_ambient']]

            def R_external_func(t):
                return 0.0

            def T_load_func(t):
                return 10.0  # Constant load

            results = sim_engine.run_simulation(
                (0, 3600),  # 1 hour
                initial_state,
                R_external_func,
                T_load_func,
                solver='RK45',
                max_step=1.0
            )

            # Update thermal plots
            self.update_thermal_plots(results, motor)

            messagebox.showinfo("Success", "Thermal simulation completed!")

        except Exception as e:
            messagebox.showerror("Error", f"Thermal simulation error: {str(e)}")

    def update_thermal_plots(self, results, motor):
        """Update thermal analysis plots"""
        # Clear plots
        self.thermal_ax1.clear()
        self.thermal_ax2.clear()
        self.thermal_ax3.clear()
        self.thermal_ax4.clear()

        # Temperature vs time
        self.thermal_ax1.plot(results['t'], results['T_arm'], 'r-', label='Armature', linewidth=2)
        self.thermal_ax1.plot(results['t'], results['T_field'], 'b-', label='Field', linewidth=2)
        self.thermal_ax1.plot(results['t'], results['T_housing'], 'g-', label='Housing', linewidth=2)
        self.thermal_ax1.axhline(y=self.params['T_max_armature'], color='k', linestyle='--', label='Max Temp')
        self.thermal_ax1.set_title('Temperature vs Time')
        self.thermal_ax1.set_xlabel('Time (s)')
        self.thermal_ax1.set_ylabel('Temperature (°C)')
        self.thermal_ax1.legend()
        self.thermal_ax1.grid(True)

        # Power losses breakdown
        if hasattr(motor, 'last_losses'):
            losses = motor.last_losses
            labels = ['Copper\n(Armature)', 'Copper\n(Field)', 'Iron', 'Mechanical', 'Stray']
            values = [losses['P_copper_arm'], losses['P_copper_field'],
                     losses['P_iron'], losses['P_mechanical'], losses['P_stray']]

            colors = ['#ff9999', '#ff6666', '#66b3ff', '#99ff99', '#ffcc99']
            self.thermal_ax2.bar(labels, values, color=colors)
            self.thermal_ax2.set_title('Power Losses Breakdown')
            self.thermal_ax2.set_ylabel('Power Loss (W)')
            self.thermal_ax2.tick_params(axis='x', rotation=0)

        # Derating curve
        T_ambient_range = np.linspace(0, 60, 100)
        # Simplified derating: 100% at 25°C, decreasing linearly
        T_ref = 25
        T_max = self.params['T_max_armature']
        derating = np.maximum(0, 100 * (T_max - T_ambient_range) / (T_max - T_ref))

        self.thermal_ax4.plot(T_ambient_range, derating, 'r-', linewidth=2)
        self.thermal_ax4.fill_between(T_ambient_range, derating, alpha=0.3)
        self.thermal_ax4.set_title('Derating Curve')
        self.thermal_ax4.set_xlabel('Ambient Temperature (°C)')
        self.thermal_ax4.set_ylabel('Power Capacity (%)')
        self.thermal_ax4.grid(True)

        self.thermal_fig.tight_layout()
        self.thermal_canvas.draw()

    def analyze_mechanical_stress(self):
        """Analyze mechanical stresses"""
        try:
            # Update parameters
            self.params['shaft_diameter'] = float(self.mech_shaft_dia.get()) / 1000  # Convert mm to m
            self.params['bearing_capacity'] = float(self.mech_bearing.get())

            # Create motor model
            motor = DCMotorPhysicsModel(self.params)

            # Calculate stress for range of operating conditions
            Ia_range = np.linspace(0, 30, 100)
            omega_range = np.linspace(0, 200, 100)  # rad/s

            # Calculate torque range
            torque_range = []
            stress_range = []

            for Ia in Ia_range:
                If = self.params['V_rated'] / self.params['Rf']
                T = motor.Kt * If * Ia
                torque_range.append(T)

                stress = motor.calculate_mechanical_stress(Ia, 100)
                stress_range.append(stress['tau_shaft'] / 1e6)  # Convert to MPa

            # Bearing load vs speed
            bearing_loads = []
            for omega in omega_range:
                stress = motor.calculate_mechanical_stress(15, omega)  # At rated current
                bearing_loads.append(stress['F_radial'])

            # Bearing life vs load
            load_range = np.linspace(100, 5000, 100)
            bearing_life = []

            for load in load_range:
                C = self.params['bearing_capacity']
                if load > 0:
                    L10_rev = (C / load)**3 * 1e6
                    n_rpm = 1500
                    L10_hours = L10_rev / (60 * n_rpm)
                    bearing_life.append(L10_hours)
                else:
                    bearing_life.append(1e10)

            # Update plots
            self.mech_ax1.clear()
            self.mech_ax1.plot(torque_range, stress_range, 'b-', linewidth=2)
            self.mech_ax1.axhline(y=self.params['tau_yield']/1e6, color='r',
                                 linestyle='--', label='Yield Strength')
            self.mech_ax1.set_title('Shaft Shear Stress vs Torque')
            self.mech_ax1.set_xlabel('Torque (N·m)')
            self.mech_ax1.set_ylabel('Shear Stress (MPa)')
            self.mech_ax1.legend()
            self.mech_ax1.grid(True)

            self.mech_ax2.clear()
            self.mech_ax2.plot(omega_range * 60 / (2*np.pi), bearing_loads, 'r-', linewidth=2)
            self.mech_ax2.set_title('Bearing Load vs Speed')
            self.mech_ax2.set_xlabel('Speed (RPM)')
            self.mech_ax2.set_ylabel('Radial Load (N)')
            self.mech_ax2.grid(True)

            self.mech_ax3.clear()
            self.mech_ax3.plot(load_range, bearing_life, 'g-', linewidth=2)
            self.mech_ax3.set_title('Bearing Life (L10) vs Load')
            self.mech_ax3.set_xlabel('Load (N)')
            self.mech_ax3.set_ylabel('L10 Life (hours)')
            self.mech_ax3.set_yscale('log')
            self.mech_ax3.grid(True, which='both')

            # Safety factors
            self.mech_ax4.clear()
            current_stress = motor.calculate_mechanical_stress(12, 157)  # Rated conditions
            safety_factors = {
                'Shaft Stress': current_stress['safety_factor_shaft'],
                'Bearing Load': self.params['bearing_capacity'] / current_stress['F_radial']
            }

            colors = ['#66b3ff', '#99ff99']
            self.mech_ax4.bar(safety_factors.keys(), safety_factors.values(), color=colors)
            self.mech_ax4.axhline(y=1.0, color='r', linestyle='--', label='Minimum')
            self.mech_ax4.set_title('Safety Factors at Rated Conditions')
            self.mech_ax4.set_ylabel('Safety Factor')
            self.mech_ax4.legend()

            self.mech_fig.tight_layout()
            self.mech_canvas.draw()

            messagebox.showinfo("Success", "Mechanical stress analysis completed!")

        except Exception as e:
            messagebox.showerror("Error", f"Mechanical analysis error: {str(e)}")

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            # Get parameters
            elec_cost = float(self.econ_elec_cost.get())
            hours = float(self.econ_hours.get())
            power_kW = float(self.econ_power.get())
            efficiency = float(self.econ_efficiency.get())
            initial_cost = float(self.econ_initial.get())
            maint_cost = float(self.econ_maint.get())
            lifetime = int(float(self.econ_lifetime.get()))
            discount_rate = float(self.econ_discount.get()) / 100

            # Operating cost analysis
            operating = EconomicAnalyzer.calculate_operating_cost(
                power_kW, hours, elec_cost, efficiency
            )

            # Lifecycle cost analysis
            lifecycle = EconomicAnalyzer.lifecycle_cost_analysis(
                initial_cost, operating['annual_cost'], maint_cost,
                lifetime, discount_rate
            )

            # Display results
            result_text = "=" * 70 + "\n"
            result_text += "ECONOMIC ANALYSIS RESULTS\n"
            result_text += "=" * 70 + "\n\n"

            result_text += "OPERATING COST ANALYSIS:\n"
            result_text += "-" * 70 + "\n"
            result_text += f"  Annual Energy Consumption  : {operating['energy_kWh']:,.1f} kWh\n"
            result_text += f"  Daily Operating Cost       : ${operating['daily_cost']:.2f}\n"
            result_text += f"  Monthly Operating Cost     : ${operating['monthly_cost']:.2f}\n"
            result_text += f"  Annual Operating Cost      : ${operating['annual_cost']:,.2f}\n\n"

            result_text += "LIFECYCLE COST ANALYSIS:\n"
            result_text += "-" * 70 + "\n"
            result_text += f"  Initial Investment         : ${lifecycle['initial_cost']:,.2f}\n"
            result_text += f"  NPV of Operating Costs     : ${lifecycle['npv_operating']:,.2f}\n"
            result_text += f"  NPV of Maintenance Costs   : ${lifecycle['npv_maintenance']:,.2f}\n"
            result_text += f"  Total Net Present Value    : ${lifecycle['total_npv']:,.2f}\n"
            result_text += f"  Annual Equivalent Cost     : ${lifecycle['annual_equivalent']:,.2f}\n\n"

            result_text += "COST BREAKDOWN:\n"
            result_text += "-" * 70 + "\n"
            pct_initial = lifecycle['initial_cost'] / lifecycle['total_npv'] * 100
            pct_operating = lifecycle['npv_operating'] / lifecycle['total_npv'] * 100
            pct_maint = lifecycle['npv_maintenance'] / lifecycle['total_npv'] * 100

            result_text += f"  Initial Cost               : {pct_initial:.1f}%\n"
            result_text += f"  Operating Cost             : {pct_operating:.1f}%\n"
            result_text += f"  Maintenance Cost           : {pct_maint:.1f}%\n\n"

            result_text += "PAYBACK ANALYSIS:\n"
            result_text += "-" * 70 + "\n"
            result_text += f"  Total cost over {lifetime} years: ${lifecycle['total_npv']:,.2f}\n"
            result_text += f"  Average cost per year      : ${lifecycle['annual_equivalent']:,.2f}\n"
            result_text += f"  Cost per operating hour    : ${lifecycle['annual_equivalent']/hours:.2f}\n"

            result_text += "\n" + "=" * 70 + "\n"

            self.econ_results_text.delete('1.0', tk.END)
            self.econ_results_text.insert('1.0', result_text)

            # Update plots
            self.econ_ax1.clear()
            labels = ['Initial\nCost', 'Operating\nCost', 'Maintenance\nCost']
            values = [pct_initial, pct_operating, pct_maint]
            colors = ['#ff9999', '#66b3ff', '#99ff99']

            self.econ_ax1.pie(values, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
            self.econ_ax1.set_title('Cost Breakdown (% of Total NPV)')

            # Cumulative cost
            self.econ_ax2.clear()
            years = np.arange(0, lifetime + 1)
            cumulative_cost = [initial_cost]

            for year in range(1, lifetime + 1):
                discount_factor = 1 / (1 + discount_rate)**year
                year_cost = (operating['annual_cost'] + maint_cost) * discount_factor
                cumulative_cost.append(cumulative_cost[-1] + year_cost)

            self.econ_ax2.plot(years, cumulative_cost, 'b-', linewidth=2, marker='o')
            self.econ_ax2.fill_between(years, cumulative_cost, alpha=0.3)
            self.econ_ax2.set_title('Cumulative Cost vs Time (NPV)')
            self.econ_ax2.set_xlabel('Year')
            self.econ_ax2.set_ylabel('Cumulative Cost ($)')
            self.econ_ax2.grid(True)

            self.econ_fig.tight_layout()
            self.econ_canvas.draw()

        except Exception as e:
            messagebox.showerror("Error", f"Economic calculation error: {str(e)}")

    def apply_control(self):
        """Apply selected control method"""
        method = self.control_method.get()

        if method == 'open_loop':
            messagebox.showinfo("Control Method",
                              "Open Loop Direct Start selected.\n"
                              "Go to Simulation tab and set External R = 0 Ω")

        elif method == 'stepped':
            messagebox.showinfo("Control Method",
                              "Stepped Resistance Starter selected.\n"
                              "Use Starter Calculator tab to design resistances.")

        elif method == 'soft_start':
            messagebox.showinfo("Control Method",
                              "Soft Start method selected.\n"
                              "This requires voltage control implementation.")

        elif method == 'pid':
            try:
                Kp = float(self.pid_kp.get())
                Ki = float(self.pid_ki.get())
                Kd = float(self.pid_kd.get())
                setpoint = float(self.pid_setpoint.get())

                messagebox.showinfo("PID Control",
                                  f"PID Parameters Set:\n"
                                  f"Kp = {Kp}\n"
                                  f"Ki = {Ki}\n"
                                  f"Kd = {Kd}\n"
                                  f"Setpoint = {setpoint} RPM\n\n"
                                  f"Note: Full PID implementation requires "
                                  f"closed-loop simulation.")
            except:
                messagebox.showerror("Error", "Invalid PID parameters!")


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = AdvancedDCMotorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
