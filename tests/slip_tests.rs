use std::f32::consts::FRAC_PI_2;
use tlab_vehicle_physics::slip::SlipCalculator;

#[test]
fn test_longitudinal_slip_ratio_driving_and_braking() {
    let calc = SlipCalculator::default();
    let r = 0.35; // 35 cm wheel radius

    // Free rolling: omega * r = vx => 10 m/s / 0.35 m = 28.57 rad/s
    let omega_roll = 10.0 / r;
    let slip_free = calc.calculate_slip_ratio(omega_roll, r, 10.0);
    assert!(slip_free.abs() < 1e-4);

    // Driving wheelspin: wheel turns twice as fast as vehicle (20 m/s surface vs 10 m/s car)
    let omega_spin = 20.0 / r;
    let slip_drive = calc.calculate_slip_ratio(omega_spin, r, 10.0);
    // kappa = (20 - 10) / 10 = +1.0 (100% positive slip)
    assert!((slip_drive - 1.0).abs() < 1e-4);

    // Locked wheel braking: omega = 0 while vehicle is moving at 10 m/s
    let slip_locked = calc.calculate_slip_ratio(0.0, r, 10.0);
    // kappa = (0 - 10) / 10 = -1.0 (-100% negative slip)
    assert!((slip_locked - (-1.0)).abs() < 1e-4);
}

#[test]
fn test_slip_ratio_zero_speed_regularization() {
    let calc = SlipCalculator::default();
    let r = 0.35;

    // Zero car velocity and zero wheel spin => cleanly 0.0 without NaN
    let slip_zero = calc.calculate_slip_ratio(0.0, r, 0.0);
    assert_eq!(slip_zero, 0.0);
    assert!(!slip_zero.is_nan());

    // Standing burnout: vehicle at rest (vx = 0), wheel spinning at 50 rad/s
    let slip_burnout = calc.calculate_slip_ratio(50.0, r, 0.0);
    assert!(!slip_burnout.is_nan());
    assert!(slip_burnout >= 0.0);
}

#[test]
fn test_lateral_slip_angle() {
    let calc = SlipCalculator::default();

    // Straight line: vy = 0, vx = 20 m/s => alpha = 0
    let alpha_straight = calc.calculate_slip_angle_rad(0.0, 20.0);
    assert_eq!(alpha_straight, 0.0);

    // Small lateral drift: vx = 20, vy = 2 => alpha = atan(2 / 20) = 0.09967 rad (~5.71 deg)
    let alpha_drift = calc.calculate_slip_angle_rad(2.0, 20.0);
    assert!((alpha_drift - 0.099668).abs() < 1e-4);

    // Pure sideways slide: vx = 0, vy = 5 => alpha = 90 degrees (pi/2)
    let alpha_sideways = calc.calculate_slip_angle_rad(5.0, 0.0);
    assert!((alpha_sideways - FRAC_PI_2).abs() < 1e-4);

    // Pure sideways left: vx = 0, vy = -5 => alpha = -90 degrees (-pi/2)
    let alpha_left = calc.calculate_slip_angle_rad(-5.0, 0.0);
    assert!((alpha_left - (-FRAC_PI_2)).abs() < 1e-4);
}

#[test]
fn test_combined_slip_state_evaluator() {
    let calc = SlipCalculator::default();
    let state = calc.calculate(35.0, 0.35, 10.0, 1.0);

    assert!(state.slip_ratio > 0.0);
    assert!(state.slip_angle_rad > 0.0);
    assert!(state.slip_angle_deg > 0.0);
    assert!(state.combined_slip_magnitude > 0.0);
}
