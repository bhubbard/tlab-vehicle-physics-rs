//! Analytical Accuracy & Numerical Parity Benchmark Tests
//! Evaluates Pacejka Magic Formula curve precision, friction circle limits, and low-speed stability.

use approx::assert_relative_eq;
use tlab_vehicle_physics::combined_slip::{CombinedSlipModel, FrictionCircleConfig};
use tlab_vehicle_physics::pacejka::{Pacejka, PacejkaCoefficients};

#[test]
fn test_pacejka_analytical_magic_formula_exactness() {
    let b = 10.0f32;
    let c = 1.30f32;
    let d = 4500.0f32; // 4.5 kN peak lateral force
    let e = -1.20f32;
    let sh = 0.015f32;
    let sv = -50.0f32;

    let p = Pacejka::new(PacejkaCoefficients {
        b,
        c,
        d,
        e,
        sh,
        sv,
    });

    let mut sum_sq_err = 0.0f64;
    let n = 1000;

    for i in 0..=n {
        let x = -0.5f32 + (i as f32 / n as f32) * 1.0f32; // slip angle -0.5 to +0.5 rad

        // Analytical ground truth:
        let x_shifted = x + sh;
        let bx = b * x_shifted;
        let analytical = d * (c * (bx - e * (bx - bx.atan())).atan()).sin() + sv;

        let computed = p.evaluate(x);
        let diff = (computed - analytical) as f64;
        sum_sq_err += diff * diff;

        assert_relative_eq!(computed, analytical, epsilon = 1e-4);
    }

    let rmse = (sum_sq_err / (n as f64 + 1.0)).sqrt();
    assert!(
        rmse < 1e-5,
        "Pacejka formula RMSE exceeded target: rmse = {rmse}"
    );
}

#[test]
fn test_friction_circle_invariance() {
    let config = FrictionCircleConfig {
        peak_friction: 1.0,
        slide_friction: 0.8,
        nominal_load: 4000.0,
        load_sensitivity: 0.0,
        ..Default::default()
    };
    let model = CombinedSlipModel::new(config);
    let fz = 4000.0f32;

    // Test a grid of slip demands
    for fx_req in [500.0, 1500.0, 3000.0, 6000.0, 10000.0] {
        for fy_req in [500.0, 1500.0, 3000.0, 6000.0, 10000.0] {
            let res = model.combine_forces(fx_req, fy_req, fz, 0.05, 0.02);
            let force_magnitude = res.total_force.length();
            let max_allowed = config.peak_friction * fz + 1e-3;

            assert!(
                force_magnitude <= max_allowed,
                "Force magnitude {force_magnitude} exceeded friction envelope {max_allowed}"
            );
        }
    }
}

#[test]
fn test_cornering_stiffness_and_peak_force() {
    let b = 12.0f32;
    let c = 1.35f32;
    let d = 3200.0f32;
    let e = -0.8f32;

    let p = Pacejka::with_bcde(b, c, d, e);

    // Theoretical cornering stiffness: S = B * C * D
    let expected_stiffness = b * c * d;
    assert_relative_eq!(p.cornering_stiffness(), expected_stiffness, epsilon = 1e-3);

    // Initial derivative numerical limit: lim_{x->0} (y/x)
    let eps = 1e-4f32;
    let numerical_stiffness = p.evaluate(eps) / eps;
    assert_relative_eq!(numerical_stiffness, expected_stiffness, epsilon = 1.0);

    // Maximum force magnitude cannot exceed D (since |sin(z)| <= 1)
    for i in 0..500 {
        let x = (i as f32) * 0.002;
        let force = p.evaluate(x);
        assert!(
            force <= d + 1e-3,
            "Force exceeded maximum peak D: force={force}, D={d}"
        );
    }
}
