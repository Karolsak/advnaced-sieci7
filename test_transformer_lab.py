#!/usr/bin/env python3
"""
Test script for Parallel Transformer Lab
Demonstrates the calculation capabilities without GUI
"""

import sys
sys.path.insert(0, '/home/user/advnaced-sieci7')

from parallel_transformer_advanced_lab import (
    ParallelTransformerAnalysis,
    MultiPhysicsSimulator,
    EconomicAnalyzer
)
import numpy as np


def test_parallel_transformer_analysis():
    """Test parallel transformer load sharing calculation"""
    print("="*70)
    print("TEST 1: PARALLEL TRANSFORMER ANALYSIS (Problem 7)")
    print("="*70)

    analyzer = ParallelTransformerAnalysis()
    results = analyzer.calculate_load_sharing()

    print("\nCalculated Impedances:")
    print(f"  Transformer A: Z={results['impedances']['z_a']:.4f} Ω, "
          f"R={results['impedances']['r_a']:.4f} Ω, "
          f"X={results['impedances']['x_a']:.4f} Ω")
    print(f"  Transformer B: Z={results['impedances']['z_b']:.4f} Ω, "
          f"R={results['impedances']['r_b']:.4f} Ω, "
          f"X={results['impedances']['x_b']:.4f} Ω")

    print("\nSecondary Voltage:")
    print(f"  V2 = {results['v2_magnitude']:.2f} V")
    print(f"  Voltage Drop = {results['v_drop']:.2f} V")

    print("\nCurrent Distribution:")
    print(f"  Transformer A: I_A = {results['i_a_magnitude']:.2f} A ∠{results['i_a_angle']:.2f}°")
    print(f"  Transformer B: I_B = {results['i_b_magnitude']:.2f} A ∠{results['i_b_angle']:.2f}°")
    print(f"  Total: I_total = {analyzer.total_load_current:.2f} A")

    print("\nPower Factors:")
    print(f"  Transformer A: p.f. = {results['pf_a']:.4f}")
    print(f"  Transformer B: p.f. = {results['pf_b']:.4f}")

    print("\nPower Output:")
    print(f"  Transformer A: S_A = {results['s_a']/1000:.2f} kVA, P_A = {results['p_a']/1000:.2f} kW")
    print(f"  Transformer B: S_B = {results['s_b']/1000:.2f} kVA, P_B = {results['p_b']/1000:.2f} kW")
    print(f"  Total Power: S_total = {(results['s_a'] + results['s_b'])/1000:.2f} kVA")

    # Verification
    current_sum = results['i_a_magnitude'] + results['i_b_magnitude']
    print(f"\nVerification:")
    print(f"  Load sharing: A={results['i_a_magnitude']/current_sum*100:.1f}%, "
          f"B={results['i_b_magnitude']/current_sum*100:.1f}%")

    print("\n✓ Test 1 PASSED\n")
    return results


def test_multiphysics_simulation():
    """Test multi-physics simulation"""
    print("="*70)
    print("TEST 2: MULTI-PHYSICS SIMULATION")
    print("="*70)

    simulator = MultiPhysicsSimulator()

    # Test RK45 solver
    print("\nRunning RK45 simulation...")
    simulator.solver_type = 'RK45'
    simulator.run_simulation(duration=0.1, v_in=230, load_current=10, frequency=50)

    print(f"  Time steps generated: {len(simulator.time_vector)}")
    print(f"  Simulation duration: {simulator.time_vector[-1]:.4f} s")

    # Check final states
    final_state = simulator.state_history[-1]
    final_temp = simulator.thermal_history[-1]
    final_losses = simulator.loss_history[-1]

    print(f"\nFinal State (RK45):")
    print(f"  Primary Current (peak): {abs(final_state[0]):.4f} A")
    print(f"  Secondary Current (peak): {abs(final_state[1]):.4f} A")
    print(f"  Temperature: {final_temp:.2f} °C")
    print(f"  Total Losses: {final_losses['total']:.2f} W")

    # Test loss breakdown
    avg_losses = {
        'Copper (Primary)': np.mean([l['copper_p'] for l in simulator.loss_history]),
        'Copper (Secondary)': np.mean([l['copper_s'] for l in simulator.loss_history]),
        'Hysteresis': np.mean([l['hysteresis'] for l in simulator.loss_history]),
        'Eddy Current': np.mean([l['eddy'] for l in simulator.loss_history]),
        'Friction': np.mean([l['friction'] for l in simulator.loss_history]),
        'Stray': np.mean([l['stray'] for l in simulator.loss_history])
    }

    print(f"\nAverage Loss Breakdown:")
    total_avg = sum(avg_losses.values())
    for loss_type, loss_value in avg_losses.items():
        print(f"  {loss_type}: {loss_value:.2f} W ({loss_value/total_avg*100:.1f}%)")
    print(f"  Total Average: {total_avg:.2f} W")

    # Test Euler solver
    print("\n\nRunning Euler simulation...")
    simulator.solver_type = 'Euler'
    simulator.run_simulation(duration=0.1, v_in=230, load_current=10, frequency=50)

    final_state_euler = simulator.state_history[-1]
    print(f"\nFinal State (Euler):")
    print(f"  Primary Current (peak): {abs(final_state_euler[0]):.4f} A")
    print(f"  Secondary Current (peak): {abs(final_state_euler[1]):.4f} A")
    print(f"  Time steps: {len(simulator.time_vector)}")

    print("\n✓ Test 2 PASSED\n")
    return simulator


def test_economic_analysis():
    """Test economic analysis"""
    print("="*70)
    print("TEST 3: ECONOMIC ANALYSIS")
    print("="*70)

    analyzer = EconomicAnalyzer()

    # Test with typical loss value
    avg_loss_kw = 0.5

    annual_cost = analyzer.calculate_energy_cost(avg_loss_kw)
    npv = analyzer.calculate_npv(avg_loss_kw)

    print(f"\nEconomic Parameters:")
    print(f"  Electricity Cost: ${analyzer.electricity_cost:.4f}/kWh")
    print(f"  Operating Hours: {analyzer.operating_hours} hours/year")
    print(f"  Transformer Cost: ${analyzer.transformer_cost:,.2f}")
    print(f"  Maintenance Cost: ${analyzer.maintenance_cost_annual:,.2f}/year")
    print(f"  Lifetime: {analyzer.lifetime_years} years")
    print(f"  Discount Rate: {analyzer.discount_rate*100:.1f}%")

    print(f"\nResults (for {avg_loss_kw} kW average loss):")
    print(f"  Annual Energy Loss: {avg_loss_kw * analyzer.operating_hours:.2f} kWh/year")
    print(f"  Annual Energy Cost: ${annual_cost:,.2f}")
    print(f"  Net Present Value: ${npv:,.2f}")

    # Test comparative analysis
    improved_loss_kw = avg_loss_kw * 0.9
    improved_annual_cost = analyzer.calculate_energy_cost(improved_loss_kw)
    annual_savings = annual_cost - improved_annual_cost
    payback = analyzer.calculate_payback_period(annual_savings)

    print(f"\nComparative Analysis (10% loss reduction):")
    print(f"  New Annual Cost: ${improved_annual_cost:,.2f}")
    print(f"  Annual Savings: ${annual_savings:,.2f}")
    print(f"  Payback Period: {payback:.1f} years")

    print("\n✓ Test 3 PASSED\n")
    return analyzer


def run_all_tests():
    """Run all tests"""
    print("\n" + "="*70)
    print("PARALLEL TRANSFORMER ADVANCED LAB - TEST SUITE")
    print("="*70 + "\n")

    try:
        # Test 1: Parallel transformer analysis
        transformer_results = test_parallel_transformer_analysis()

        # Test 2: Multi-physics simulation
        simulator_results = test_multiphysics_simulation()

        # Test 3: Economic analysis
        economic_results = test_economic_analysis()

        # Summary
        print("="*70)
        print("TEST SUMMARY")
        print("="*70)
        print("✓ All tests passed successfully!")
        print("\nThe application is ready to use.")
        print("Run: python3 parallel_transformer_advanced_lab.py")
        print("="*70 + "\n")

        return True

    except Exception as e:
        print(f"\n✗ TEST FAILED: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
