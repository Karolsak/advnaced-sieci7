# Advanced DC Motor Starter & Multi-Physics Simulator

## Overview

This comprehensive Python application provides advanced simulation and analysis tools for DC motors, including:

- **Starter Resistance Calculations** (Problems 18 & 19)
- **Multi-Physics Simulation** (Electromagnetic-Thermal-Mechanical coupling)
- **Real-time Dynamic Simulation** with multiple ODE solvers
- **Economic Analysis** and lifecycle costing
- **Advanced Control Methods**
- **Thermal and Derating Analysis**
- **Mechanical Stress Analysis**

## Installation Requirements

```bash
pip install numpy scipy matplotlib tkinter
```

## Running the Application

```bash
python3 dc_motor_starter_simulator.py
```

## Features

### 1. Starter Calculator Tab

#### Problem 18: 6-Stud Starter Design
Calculates the resistance values for a 5-step, 6-stud starter to limit starting current.

**Inputs:**
- Supply Voltage (V)
- Full-load Current (A)
- Armature Resistance Ra (Ω)
- Field Resistance Rf (Ω)
- Maximum Current Factor

**Outputs:**
- Five resistance values for each starter step
- Current ratio (α)
- Detailed calculation breakdown

**Expected Results:** [6.57 Ω, 3.12 Ω, 1.48 Ω, 0.7 Ω, 0.33 Ω]

#### Problem 19: Resistance Between Studs
Calculates the resistance between two consecutive starter studs.

**Inputs:**
- Supply Voltage (V)
- Maximum current on first stud (A)
- Minimum current on first stud (A)
- Maximum current on second stud (A)
- Armature Resistance Ra (Ω)

**Outputs:**
- Resistance between first and second studs
- Back EMF calculations
- Step-by-step solution

### 2. Dynamic Simulation Tab

Real-time motor simulation with adjustable parameters:

**Controls:**
- **Voltage Slider:** Adjust supply voltage (0-250V)
- **Load Torque Slider:** Set mechanical load (0-50 N·m)
- **External Resistance Slider:** Add starting resistance (0-10 Ω)
- **Simulation Time:** Duration in seconds
- **Solver Selection:** Choose between RK45 (accurate) or Euler (fast)

**Buttons:**
- **Start:** Begin simulation
- **Stop:** Halt running simulation
- **Reset:** Clear results and reset

**Visualizations:**
- Armature current vs time
- Motor speed (RPM) vs time
- Electromagnetic torque vs time
- Efficiency vs time

**Mathematical Model:**
The simulator solves coupled differential equations:

```
Electrical: dIa/dt = (V - Eb - Ia*(Ra + R_ext)) / La
Mechanical: dω/dt = (T_em - T_load - B*ω) / J
Thermal: dT/dt = (Q_gen - Q_loss) / C
```

Where:
- Ia = Armature current
- ω = Angular velocity
- Eb = Back EMF = Ka * ω
- T_em = Electromagnetic torque = Kt * If * Ia
- La = Armature inductance
- J = Total moment of inertia
- B = Friction coefficient

### 3. Thermal Analysis Tab

Multi-component thermal modeling with heat transfer:

**Components Modeled:**
- Armature winding temperature
- Field winding temperature
- Motor housing temperature

**Loss Calculations:**
- Copper losses (I²R) in armature and field
- Iron losses (hysteresis and eddy currents)
- Mechanical friction losses
- Stray load losses

**Visualizations:**
- Temperature vs time for all components
- Power losses breakdown (pie chart)
- Heat flow diagram
- Derating curve (power capacity vs ambient temperature)

**Thermal Model:**
Uses lumped-parameter thermal network:
- Thermal capacitances for each component
- Thermal resistances for heat transfer paths
- Convection to ambient

### 4. Mechanical Analysis Tab

Shaft and bearing stress analysis:

**Calculations:**
- **Shaft Shear Stress:** τ = T*r/J_polar
- **Bearing Radial Load:** Includes rotor weight and belt tension
- **Bearing Life:** L10 life calculation using ISO 281 standard
- **Safety Factors:** For shaft and bearings

**Visualizations:**
- Shaft stress vs torque
- Bearing load vs speed
- Bearing life vs load (logarithmic scale)
- Safety factors at rated conditions

### 5. Economic Analysis Tab

Comprehensive lifecycle cost analysis:

**Operating Cost Analysis:**
- Annual energy consumption (kWh)
- Daily, monthly, and annual operating costs
- Accounts for motor efficiency

**Lifecycle Cost Analysis (NPV):**
- Initial capital investment
- Net present value of operating costs
- Net present value of maintenance costs
- Total NPV and annual equivalent cost

**Visualizations:**
- Cost breakdown pie chart
- Cumulative cost vs time

**Formulas:**
```
Annual Energy = (P_avg * Hours) / (η/100)
NPV = Initial + Σ(Annual_Cost / (1+r)^t)
Annual Equivalent = NPV * r / (1 - (1+r)^(-n))
```

### 6. Advanced Controls Tab

Multiple motor starting and control methods:

**Control Methods:**

1. **Open Loop (Direct Start)**
   - Direct connection to supply
   - Highest starting current (5-8x rated)
   - Simple but stressful on motor

2. **Stepped Resistance Starter**
   - Progressive resistance reduction
   - Limits starting current to 1.5-2.5x rated
   - Calculated using Problem 18 results

3. **Soft Start (Voltage Ramp)**
   - Gradual voltage increase
   - Smooth acceleration
   - Requires power electronics

4. **PID Speed Control**
   - Closed-loop speed regulation
   - Tunable parameters (Kp, Ki, Kd)
   - Compensates for load variations

**PID Parameters:**
- Kp: Proportional gain (response speed)
- Ki: Integral gain (steady-state error elimination)
- Kd: Derivative gain (overshoot reduction)

## Multi-Physics Simulation Details

### Electromagnetic Model
- Kirchhoff's voltage law for DC motor
- Back EMF proportional to speed
- Torque-current relationship
- RMS voltage and current used

### Thermal Model
- Three-node thermal network
- Heat generation from all loss sources
- Convective heat transfer
- Thermal time constants

### Mechanical Model
- Newton's second law of rotation
- Inertia of rotor and load
- Viscous friction
- Shaft stress analysis
- Bearing dynamics

### Coupling Effects
- Temperature affects resistance (R = R₀(1 + α*ΔT))
- Speed affects iron losses (proportional to f^1.5)
- Thermal limits affect power rating
- Mechanical stress limits torque capacity

## ODE Solvers

### RK45 (Runge-Kutta 4th/5th order)
- Adaptive step size
- High accuracy
- Recommended for precise results
- Slower computation

### Euler (Explicit Euler)
- Fixed step size
- Fast computation
- Lower accuracy
- Good for educational purposes

## Auto-Scaling Features

The GUI automatically adjusts to window size changes:
- Grid weights configured for responsive layout
- Plots scale with available space
- All tabs support resizing

## Practical Applications

1. **Motor Selection:** Compare different motors based on lifecycle costs
2. **Starter Design:** Calculate optimal starter resistances
3. **Thermal Management:** Predict operating temperatures
4. **Maintenance Planning:** Estimate bearing life
5. **Energy Audits:** Calculate operating costs
6. **Control System Design:** Tune PID controllers
7. **Educational Tool:** Understand motor dynamics

## Technical Specifications

**Simulation Parameters (Default):**
- Rated Voltage: 200 V
- Rated Current: 12 A
- Rated Speed: 1500 RPM
- Rated Power: 2 kW
- Armature Resistance: 0.3 Ω
- Field Resistance: 100 Ω
- Armature Inductance: 0.05 H
- Rotor Inertia: 0.05 kg·m²
- Maximum Temperature: 155°C (Class F insulation)

## Usage Examples

### Example 1: Design a Starter
1. Go to "Starter Calculator" tab
2. Enter motor parameters
3. Click "Calculate" for Problem 18
4. Use results to specify external resistances

### Example 2: Simulate Motor Starting
1. Go to "Dynamic Simulation" tab
2. Set External R to calculated starter resistance
3. Set load torque
4. Click "Start"
5. Observe current, speed, and torque transients

### Example 3: Check Thermal Limits
1. Go to "Thermal Analysis" tab
2. Set ambient temperature
3. Click "Run Thermal Simulation"
4. Verify temperatures stay below limits

### Example 4: Economic Comparison
1. Go to "Economic Analysis" tab
2. Enter electricity cost and operating hours
3. Click "Calculate Economics"
4. Review lifecycle costs

## Troubleshooting

**Issue:** Simulation runs slowly
- **Solution:** Use Euler solver or increase max_step

**Issue:** Temperature exceeds limits
- **Solution:** Reduce load torque or improve cooling

**Issue:** Plots not updating
- **Solution:** Click "Reset" and restart simulation

**Issue:** High starting current
- **Solution:** Add external resistance or use soft start

## Mathematical Background

### DC Motor Equations

**Voltage Equation:**
```
V = Eb + Ia*Ra + La*(dIa/dt)
```

**Back EMF:**
```
Eb = Ka * ω
```

**Torque Equation:**
```
T_em = Kt * Φ * Ia = Kt * If * Ia  (for shunt motor)
```

**Mechanical Equation:**
```
J*(dω/dt) = T_em - T_load - B*ω
```

**Thermal Equation:**
```
C*(dT/dt) = P_loss - (T - T_amb)/R_th
```

### Loss Breakdown

1. **Copper Losses:** Pcu = I²R (in armature and field)
2. **Iron Losses:** Pfe = Kh*f*B² + Ke*f²*B² (hysteresis + eddy)
3. **Mechanical Losses:** Pmech = B*ω² (friction and windage)
4. **Stray Losses:** Pstray ≈ 1% of output power

### Efficiency

```
η = P_out / P_in = (T_em * ω) / (V * Ia)
```

## References

1. Chapman, S.J. "Electric Machinery Fundamentals"
2. Sen, P.C. "Principles of Electric Machines and Power Electronics"
3. Fitzgerald, A.E. "Electric Machinery"
4. IEEE Standard 112: "Test Procedure for Polyphase Induction Motors and Generators"

## License

This software is provided for educational and research purposes.

## Author

Advanced DC Motor Simulator v1.0
Created: 2025

## Contact

For issues, questions, or contributions, please refer to the documentation.
