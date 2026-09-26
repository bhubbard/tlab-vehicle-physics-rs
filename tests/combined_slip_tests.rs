use tlab_vehicle_physics::combined_slip::{CombinedSlipModel, FrictionCircleConfig};

#[test]
fn test_normal_load_sensitivity() {
    let config = FrictionCircleConfig {
        peak_friction: 1.20,
        nominal_load: 3000.0,
        load_sensitivity: 0.15,
        ..Default::default()
    };
    let model = CombinedSlipModel::new(config);

    // At nominal load: mu = peak_friction (1.20)
    let mu_nom = model.load_sensitive_mu(3000.0);
    assert!((mu_nom - 1.20).abs() < 1e-4);

    // Under high load (6000 N, double nominal): mu decreases due to tire load sensitivity
    let mu_high = model.load_sensitive_mu(6000.0);
    assert!(mu_high < mu_nom);

    // Under light load (1500 N, half nominal): mu is higher
    let mu_light = model.load_sensitive_mu(1500.0);
    assert!(mu_light > mu_nom);
}

#[test]
fn test_friction_circle_limit() {
    let config = FrictionCircleConfig {
        peak_friction: 1.0,
        slide_friction: 0.8,
        nominal_load: 4000.0,
        load_sensitivity: 0.0, // Disable load sensitivity for pure circle test
        ..Default::default()
    };
    let model = CombinedSlipModel::new(config);
    let fz = 4000.0;

    // Small demand inside circle: fx = 1500 N, fy = 2000 N
    // Resultant = sqrt(1500^2 + 2000^2) = 2500 N <= 4000 N (within grip)
    let result_inside = model.combine_forces(1500.0, 2000.0, fz, 0.0, 0.0);
    assert!(!result_inside.is_sliding);
    assert!((result_inside.total_force.length() - 2500.0).abs() < 1e-3);

    // Huge demand outside circle: fx = 6000 N, fy = 6000 N => requested ~8100 N > 4000 N
    let result_sliding = model.combine_forces(6000.0, 6000.0, fz, 0.05, 0.02);
    assert!(result_sliding.is_sliding);
    assert!(result_sliding.grip_utilization > 1.0);

    // Resultant force must be clamped within available friction limit
    let clamped_len = result_sliding.total_force.length();
    assert!(clamped_len <= config.peak_friction * fz);
    assert!(clamped_len >= config.slide_friction * fz * 0.95);
}

#[test]
fn test_pacejka_combined_slip_weighting() {
    let model = CombinedSlipModel::new(FrictionCircleConfig::default());

    // When slip angle is 0, g_xa multiplier is 1.0 (no reduction on longitudinal grip)
    let gxa_straight = model.g_xa(0.0);
    assert!((gxa_straight - 1.0).abs() < 1e-4);

    // When slip angle is large (hard cornering), longitudinal grip available is reduced
    let gxa_cornering = model.g_xa(0.20); // ~11.5 degrees slip angle
    assert!(gxa_cornering < 1.0);
    assert!(gxa_cornering > 0.0);

    // When slip ratio is 0, g_yk multiplier is 1.0 (no reduction on lateral grip)
    let gyk_no_spin = model.g_yk(0.0);
    assert!((gyk_no_spin - 1.0).abs() < 1e-4);

    // When wheel is spinning hard under throttle, lateral grip reduces
    let gyk_spinning = model.g_yk(0.50);
    assert!(gyk_spinning < 1.0);
    assert!(gyk_spinning > 0.0);
}
