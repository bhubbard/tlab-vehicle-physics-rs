use tlab_vehicle_physics::tire_forces::TireModel;

#[test]
fn test_camber_thrust() {
    let tire = TireModel::default();
    let fz = 4000.0;

    // 0 camber => zero camber thrust
    let thrust_zero = tire.calculate_camber_thrust(0.0, fz);
    assert_eq!(thrust_zero, 0.0);

    // Negative camber (-2 degrees = -0.0349 rad)
    let camber_rad = -2.0_f32.to_radians();
    let thrust_neg = tire.calculate_camber_thrust(camber_rad, fz);
    // Thrust pushes towards tilt direction (negative)
    assert!(thrust_neg < 0.0);
    // Expected: 0.08 * (-0.0349) * 4000 = -11.17 N
    let expected = tire.aligning_config.camber_stiffness * camber_rad * fz;
    assert!((thrust_neg - expected).abs() < 1e-3);
}

#[test]
fn test_pneumatic_trail_and_aligning_torque() {
    let tire = TireModel::default();

    // At zero slip angle, pneumatic trail is at maximum peak trail (~0.028m)
    let trail_zero = tire.calculate_pneumatic_trail(0.0);
    assert!((trail_zero - tire.aligning_config.peak_trail).abs() < 1e-4);

    // At moderate slip angle (e.g. 4 degrees), trail remains positive
    let trail_mid = tire.calculate_pneumatic_trail(4.0_f32.to_radians());
    assert!(trail_mid > 0.0);

    // At high slip angle (drift / slide at 20 degrees), pneumatic trail diminishes
    let trail_drift = tire.calculate_pneumatic_trail(20.0_f32.to_radians());
    assert!(trail_drift < trail_zero);
}

#[test]
fn test_full_tire_evaluation() {
    let tire = TireModel::default();
    let r = 0.34;
    let fz = 4000.0;

    // Straight driving with slight acceleration
    let out_straight = tire.evaluate(
        35.0, // omega
        r,
        11.0, // vx
        0.0,  // vy
        fz,
        0.0,  // camber
    );

    assert!(out_straight.longitudinal_force > 0.0);
    assert_eq!(out_straight.cornering_force, 0.0);
    assert_eq!(out_straight.aligning_torque, 0.0);
    assert!(!out_straight.is_sliding);

    // Cornering under steady speed
    let out_turn = tire.evaluate(
        11.0 / r,
        r,
        11.0,
        1.5, // 1.5 m/s lateral velocity
        fz,
        -1.5_f32.to_radians(),
    );

    // Cornering force opposes lateral velocity (points left / negative Y)
    assert!(out_turn.cornering_force < 0.0);
    assert!(out_turn.aligning_torque != 0.0);
    assert!(out_turn.grip_utilization > 0.0);
}

#[test]
fn test_zero_normal_load() {
    let tire = TireModel::default();
    let out = tire.evaluate(30.0, 0.34, 10.0, 2.0, 0.0, 0.0);

    assert_eq!(out.longitudinal_force, 0.0);
    assert_eq!(out.lateral_force, 0.0);
    assert_eq!(out.aligning_torque, 0.0);
    assert!(!out.is_sliding);
}
