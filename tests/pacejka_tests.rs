use tlab_vehicle_physics::pacejka::{
    MultiPacejka, MultiPacejkaElement, Pacejka, PacejkaCoefficients,
};

#[test]
fn test_pacejka_magic_formula_evaluation() {
    let p = Pacejka::with_bcde(10.0, 1.3, 1.0, -1.0);

    // At zero slip, force is zero
    assert_eq!(p.evaluate(0.0), 0.0);

    // Initial slope / cornering stiffness = B * C * D = 10 * 1.3 * 1.0 = 13.0
    assert!((p.cornering_stiffness() - 13.0).abs() < 1e-4);

    // Small slip: slope should closely match cornering stiffness
    let small_x = 0.001;
    let small_y = p.evaluate(small_x);
    let estimated_slope = small_y / small_x;
    assert!((estimated_slope - 13.0).abs() < 0.1);

    // Peak force should not exceed D = 1.0
    for i in 0..100 {
        let x = (i as f32) * 0.02;
        let y = p.evaluate(x);
        assert!(y <= 1.01);
    }
}

#[test]
fn test_pacejka_shifts() {
    let coeffs = PacejkaCoefficients {
        b: 10.0,
        c: 1.5,
        d: 1000.0,
        e: 0.0,
        sh: 0.02,  // Horizontal shift
        sv: 50.0,  // Vertical shift
    };
    let p = Pacejka::new(coeffs);

    // When x = -sh, input to curve is 0, so output is sv
    let y_at_neg_sh = p.evaluate(-0.02);
    assert!((y_at_neg_sh - 50.0).abs() < 1e-3);
}

#[test]
fn test_tlab_csharp_formula_evaluation() {
    let p = Pacejka::with_bcde(0.8, 1.0, 1.0, -2.0);

    // Zero slip = zero force
    assert_eq!(p.evaluate_tlab(0.0), 0.0);

    let val = p.evaluate_tlab(1.0);
    assert!(val > 0.0 && val <= 1.0);
}

#[test]
fn test_multi_pacejka_interpolation() {
    let p1 = Pacejka::with_bcde(10.0, 1.3, 1000.0, -1.0);
    let p2 = Pacejka::with_bcde(10.0, 1.3, 2000.0, -1.0);

    let multi = MultiPacejka::new(vec![
        MultiPacejkaElement {
            index: 2000.0, // Normal load 2000 N
            pacejka: p1,
        },
        MultiPacejkaElement {
            index: 4000.0, // Normal load 4000 N
            pacejka: p2,
        },
    ]);

    // Exact evaluation at index 2000
    let y_2000 = multi.evaluate(2000.0, 0.1);
    let exp_2000 = p1.evaluate(0.1);
    assert!((y_2000 - exp_2000).abs() < 1e-4);

    // Exact evaluation at index 4000
    let y_4000 = multi.evaluate(4000.0, 0.1);
    let exp_4000 = p2.evaluate(0.1);
    assert!((y_4000 - exp_4000).abs() < 1e-4);

    // Midway at index 3000 should be average of p1 and p2
    let y_3000 = multi.evaluate(3000.0, 0.1);
    let exp_3000 = 0.5 * (exp_2000 + exp_4000);
    assert!((y_3000 - exp_3000).abs() < 1e-4);
}
