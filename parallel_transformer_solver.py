#!/usr/bin/env python3
"""
Standalone Parallel Transformer Load Sharing Calculator
Solves Problem 7 without GUI dependencies

Problem:
Two single-phase transformers work in parallel on a load of 750 A at 0.8 p.f. lagging.
Determine secondary voltage and the output and power factor of each transformer.

Test data:
- Open circuit: 11,000/13,300 V for each transformer
- Short circuit (with h.v. winding short-circuit):
  * Transformer A: secondary input 200 V, 400 A, 15 kW
  * Transformer B: secondary input 100 V, 400 A, 20 kW
"""

import cmath
import math


class ParallelTransformerSolver:
    """Solves parallel transformer load sharing problems"""

    def __init__(self):
        # Default values for Problem 7
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

    def calculate_impedances(self):
        """
        Calculate equivalent impedances from short circuit tests

        Returns:
            dict: Dictionary containing impedance parameters
        """
        # Transformer A
        z_a = self.vsc_a / self.isc_a  # Impedance magnitude (Ω)
        r_a = self.psc_a / (self.isc_a ** 2)  # Resistance (Ω)
        x_a = math.sqrt(z_a**2 - r_a**2)  # Reactance (Ω)

        # Transformer B
        z_b = self.vsc_b / self.isc_b  # Impedance magnitude (Ω)
        r_b = self.psc_b / (self.isc_b ** 2)  # Resistance (Ω)
        x_b = math.sqrt(z_b**2 - r_b**2)  # Reactance (Ω)

        return {
            'z_a': z_a, 'r_a': r_a, 'x_a': x_a,
            'z_b': z_b, 'r_b': r_b, 'x_b': x_b
        }

    def solve(self):
        """
        Solve for load sharing between parallel transformers

        Returns:
            dict: Complete solution with all calculated values
        """
        # Calculate impedances
        imp = self.calculate_impedances()

        # Load angle (power factor lagging means current lags voltage)
        load_angle = math.acos(self.load_pf)

        # Total load current (complex form)
        # Using voltage as reference (angle = 0)
        # Lagging current has negative angle
        i_total = self.total_load_current * cmath.exp(-1j * load_angle)

        # Complex impedances
        z_a_complex = imp['r_a'] + 1j * imp['x_a']
        z_b_complex = imp['r_b'] + 1j * imp['x_b']

        # Equivalent impedance (parallel combination)
        z_eq = (z_a_complex * z_b_complex) / (z_a_complex + z_b_complex)

        # Voltage drop across equivalent impedance
        v_drop = i_total * z_eq

        # Secondary voltage (assuming nominal open circuit voltage)
        # The actual secondary voltage is the nominal voltage minus the drop
        v2 = self.v_oc_secondary - v_drop
        v2_magnitude = abs(v2)

        # Individual transformer currents (using current divider)
        # Current divides inversely proportional to impedance
        i_a = v_drop / z_a_complex
        i_b = v_drop / z_b_complex

        # Magnitudes and angles
        i_a_magnitude = abs(i_a)
        i_b_magnitude = abs(i_b)
        i_a_angle = math.degrees(cmath.phase(i_a))
        i_b_angle = math.degrees(cmath.phase(i_b))

        # Power factors for each transformer
        # pf = cos(angle between current and voltage)
        pf_a = math.cos(cmath.phase(i_a) - cmath.phase(v2))
        pf_b = math.cos(cmath.phase(i_b) - cmath.phase(v2))

        # Apparent power (S = V × I)
        s_a = v2_magnitude * i_a_magnitude
        s_b = v2_magnitude * i_b_magnitude

        # Real power (P = S × pf)
        p_a = s_a * pf_a
        p_b = s_b * pf_b

        # Reactive power (Q = S × sin(φ))
        q_a = s_a * math.sin(math.acos(pf_a))
        q_b = s_b * math.sin(math.acos(pf_b))

        return {
            'impedances': imp,
            'v2_magnitude': v2_magnitude,
            'v2_angle': math.degrees(cmath.phase(v2)),
            'i_a_magnitude': i_a_magnitude,
            'i_b_magnitude': i_b_magnitude,
            'i_a_angle': i_a_angle,
            'i_b_angle': i_b_angle,
            'pf_a': pf_a,
            'pf_b': pf_b,
            's_a': s_a,
            's_b': s_b,
            'p_a': p_a,
            'p_b': p_b,
            'q_a': q_a,
            'q_b': q_b,
            'v_drop': abs(v_drop),
            'z_eq': abs(z_eq),
            'z_eq_angle': math.degrees(cmath.phase(z_eq))
        }

    def print_solution(self):
        """Print formatted solution"""
        results = self.solve()

        print("="*80)
        print(" "*20 + "PARALLEL TRANSFORMER ANALYSIS")
        print(" "*30 + "PROBLEM 7 SOLUTION")
        print("="*80)
        print()

        print("GIVEN DATA:")
        print("-"*80)
        print(f"Open Circuit Voltage Ratio:    {self.v_oc_primary:,} V / {self.v_oc_secondary:,} V")
        print(f"Total Load:                    {self.total_load_current} A at {self.load_pf} p.f. lagging")
        print()
        print("Short Circuit Test Data:")
        print(f"  Transformer A:               {self.vsc_a} V, {self.isc_a} A, {self.psc_a/1000} kW")
        print(f"  Transformer B:               {self.vsc_b} V, {self.isc_b} A, {self.psc_b/1000} kW")
        print()

        print("STEP 1: CALCULATE IMPEDANCES")
        print("-"*80)
        imp = results['impedances']
        print(f"Transformer A:")
        print(f"  Z_A = V_sc / I_sc = {self.vsc_a} / {self.isc_a} = {imp['z_a']:.4f} Ω")
        print(f"  R_A = P_sc / I_sc² = {self.psc_a} / {self.isc_a}² = {imp['r_a']:.4f} Ω")
        print(f"  X_A = √(Z_A² - R_A²) = {imp['x_a']:.4f} Ω")
        print()
        print(f"Transformer B:")
        print(f"  Z_B = V_sc / I_sc = {self.vsc_b} / {self.isc_b} = {imp['z_b']:.4f} Ω")
        print(f"  R_B = P_sc / I_sc² = {self.psc_b} / {self.isc_b}² = {imp['r_b']:.4f} Ω")
        print(f"  X_B = √(Z_B² - R_B²) = {imp['x_b']:.4f} Ω")
        print()
        print(f"Equivalent Impedance (parallel):")
        print(f"  Z_eq = {results['z_eq']:.4f} Ω ∠{results['z_eq_angle']:.2f}°")
        print()

        print("STEP 2: SECONDARY VOLTAGE")
        print("-"*80)
        print(f"Voltage Drop:                  {results['v_drop']:.2f} V")
        print(f"Secondary Voltage (V₂):        {results['v2_magnitude']:.2f} V ∠{results['v2_angle']:.2f}°")
        print(f"Percentage Regulation:         {(results['v_drop']/self.v_oc_secondary)*100:.3f}%")
        print()

        print("STEP 3: CURRENT DISTRIBUTION")
        print("-"*80)
        print(f"Transformer A Current (I_A):   {results['i_a_magnitude']:.2f} A ∠{results['i_a_angle']:.2f}°")
        print(f"Transformer B Current (I_B):   {results['i_b_magnitude']:.2f} A ∠{results['i_b_angle']:.2f}°")
        print(f"Total Load Current:            {self.total_load_current:.2f} A")
        print()
        total_current = results['i_a_magnitude'] + results['i_b_magnitude']
        print(f"Load Sharing:")
        print(f"  Transformer A:               {(results['i_a_magnitude']/total_current)*100:.2f}%")
        print(f"  Transformer B:               {(results['i_b_magnitude']/total_current)*100:.2f}%")
        print()

        print("STEP 4: POWER FACTOR OF EACH TRANSFORMER")
        print("-"*80)
        print(f"Transformer A Power Factor:    {results['pf_a']:.4f} {'lagging' if results['pf_a'] > 0 else 'leading'}")
        print(f"Transformer B Power Factor:    {results['pf_b']:.4f} {'lagging' if results['pf_b'] > 0 else 'leading'}")
        print(f"Load Power Factor:             {self.load_pf:.4f} lagging")
        print()

        print("STEP 5: OUTPUT POWER OF EACH TRANSFORMER")
        print("-"*80)
        print(f"Transformer A:")
        print(f"  Apparent Power (S_A):        {results['s_a']/1000:.2f} kVA")
        print(f"  Real Power (P_A):            {results['p_a']/1000:.2f} kW")
        print(f"  Reactive Power (Q_A):        {results['q_a']/1000:.2f} kVAR")
        print()
        print(f"Transformer B:")
        print(f"  Apparent Power (S_B):        {results['s_b']/1000:.2f} kVA")
        print(f"  Real Power (P_B):            {results['p_b']/1000:.2f} kW")
        print(f"  Reactive Power (Q_B):        {results['q_b']/1000:.2f} kVAR")
        print()
        print(f"Total Output:")
        print(f"  Total Apparent Power:        {(results['s_a'] + results['s_b'])/1000:.2f} kVA")
        print(f"  Total Real Power:            {(results['p_a'] + results['p_b'])/1000:.2f} kW")
        print(f"  Total Reactive Power:        {(results['q_a'] + results['q_b'])/1000:.2f} kVAR")
        print()

        print("="*80)
        print(" "*30 + "FINAL ANSWERS")
        print("="*80)
        print(f"1. Secondary Voltage:          {results['v2_magnitude']:.2f} V")
        print()
        print(f"2. Transformer A Output:")
        print(f"   • Current:                  {results['i_a_magnitude']:.2f} A")
        print(f"   • Power Factor:             {results['pf_a']:.4f}")
        print(f"   • Apparent Power:           {results['s_a']/1000:.2f} kVA")
        print(f"   • Real Power:               {results['p_a']/1000:.2f} kW")
        print()
        print(f"3. Transformer B Output:")
        print(f"   • Current:                  {results['i_b_magnitude']:.2f} A")
        print(f"   • Power Factor:             {results['pf_b']:.4f}")
        print(f"   • Apparent Power:           {results['s_b']/1000:.2f} kVA")
        print(f"   • Real Power:               {results['p_b']/1000:.2f} kW")
        print("="*80)
        print()

        return results


def main():
    """Main function"""
    print()
    solver = ParallelTransformerSolver()
    results = solver.print_solution()

    # Verification
    print("VERIFICATION:")
    print("-"*80)
    # Note: Currents must be added vectorially, not algebraically
    # The vector sum I_A + I_B should equal I_total
    i_a_complex = results['i_a_magnitude'] * cmath.exp(1j * math.radians(results['i_a_angle']))
    i_b_complex = results['i_b_magnitude'] * cmath.exp(1j * math.radians(results['i_b_angle']))
    i_sum_complex = i_a_complex + i_b_complex
    i_sum_magnitude = abs(i_sum_complex)

    print(f"Scalar sum of currents: {results['i_a_magnitude'] + results['i_b_magnitude']:.2f} A")
    print(f"Vector sum of currents: {i_sum_magnitude:.2f} A")
    print(f"Given total load: {solver.total_load_current} A")
    print(f"Difference (vector): {abs(i_sum_magnitude - solver.total_load_current):.2f} A")

    if abs(i_sum_magnitude - solver.total_load_current) < 1:
        print("✓ Current balance verified (vector sum)!")
    else:
        print("⚠ Warning: Current imbalance detected")

    print()
    print("NOTE: For the complete interactive application with GUI,")
    print("      multi-physics simulation, and visualization, run:")
    print("      python3 parallel_transformer_advanced_lab.py")
    print()
    print("      (Requires: tkinter, numpy, scipy, matplotlib)")
    print()


if __name__ == "__main__":
    main()
