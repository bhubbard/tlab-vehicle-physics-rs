use std::time::Instant;
use tlab_vehicle_physics::prelude::*;

fn main() {
    println!("============================================================");
    println!(" tlab-vehicle-physics-rs (Rust) vs Unity TLabPhysics (C#)   ");
    println!("============================================================");

    // 1. Single Wheel Full Pacejka + Combined Slip + Aligning Torque
    println!("\n--- 1. Full Tire Physics Step (Pacejka, Combined Slip, Mz, Camber) ---");
    {
        let tire = TireModel::default();
        let r = 0.34;
        let fz = 4200.0; // 4.2 kN normal load
        let camber = -2.0_f32.to_radians();

        let iterations = 2_000_000;
        let start = Instant::now();
        let mut total_force = 0.0;

        for i in 0..iterations {
            let slip_mod = (i % 100) as f32 * 0.005;
            let omega = 36.0 + slip_mod;
            let vx = 12.0;
            let vy = 1.2 + slip_mod * 0.5;

            let out = tire.evaluate(omega, r, vx, vy, fz, camber);
            total_force += out.longitudinal_force + out.cornering_force;
        }

        let elapsed = start.elapsed();
        let ns_per_wheel = elapsed.as_nanos() as f64 / iterations as f64;
        let wheels_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Iterations: {} | Total Time: {:.2?} | Latency: {:.2} ns/tire | {:>10.0} tire-ticks/s | Force Sum: {:.1}",
            iterations, elapsed, ns_per_wheel, wheels_per_sec, total_force
        );
    }

    // 2. Full 4-Wheel Vehicle Rig Evaluation
    println!("\n--- 2. Full 4-Wheel Vehicle Chassis Tire Evaluation (4 Wheels) ---");
    {
        let tire_fl = TireModel::default();
        let tire_fr = TireModel::default();
        let tire_rl = TireModel::default();
        let tire_rr = TireModel::default();

        let r = 0.34;
        let fz_front = 4100.0;
        let fz_rear = 3900.0;
        let camber_f = -2.5_f32.to_radians();
        let camber_r = -1.5_f32.to_radians();

        let iterations = 1_000_000;
        let start = Instant::now();
        let mut total_aligning_torque = 0.0;

        for i in 0..iterations {
            let steer_rad = 0.05 + (i % 10) as f32 * 0.01;
            let vy = 0.8 + (i % 5) as f32 * 0.1;

            let out_fl = tire_fl.evaluate(35.0, r, 12.0, vy + steer_rad * 10.0, fz_front, camber_f);
            let out_fr = tire_fr.evaluate(35.0, r, 12.0, vy + steer_rad * 10.0, fz_front, -camber_f);
            let out_rl = tire_rl.evaluate(36.2, r, 12.0, vy, fz_rear, camber_r);
            let out_rr = tire_rr.evaluate(36.2, r, 12.0, vy, fz_rear, -camber_r);

            total_aligning_torque += out_fl.aligning_torque + out_fr.aligning_torque + out_rl.aligning_torque + out_rr.aligning_torque;
        }

        let elapsed = start.elapsed();
        let ns_per_car = elapsed.as_nanos() as f64 / iterations as f64;
        let cars_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Vehicles (4 Wheels): {} | Total Time: {:.2?} | Latency: {:.2} ns/vehicle | {:>10.0} vehicles/s | Mz Sum: {:.1}",
            iterations, elapsed, ns_per_car, cars_per_sec, total_aligning_torque
        );
    }

    // 3. Multi-Vehicle Fleet Scaling (100, 500, 1,000 Concurrent Cars)
    println!("\n--- 3. Multi-Vehicle Fleet Scaling (4 Wheels per Vehicle at 60 FPS) ---");
    for &fleet_size in &[100, 500, 1000] {
        let tire = TireModel::default();
        let r = 0.34;
        let fz = 4000.0;

        let frames = 1000;
        let start = Instant::now();

        for _ in 0..frames {
            for _ in 0..fleet_size {
                // 4 wheels per vehicle
                let _w1 = tire.evaluate(35.0, r, 12.0, 0.4, fz, -0.03);
                let _w2 = tire.evaluate(35.0, r, 12.0, 0.4, fz, 0.03);
                let _w3 = tire.evaluate(36.0, r, 12.0, 0.2, fz, -0.02);
                let _w4 = tire.evaluate(36.0, r, 12.0, 0.2, fz, 0.02);
            }
        }

        let elapsed = start.elapsed();
        let frame_latency = elapsed / frames as u32;
        let fps_capacity = frames as f64 / elapsed.as_secs_f64();
        let percent_60fps = (frame_latency.as_secs_f64() / 0.016666) * 100.0;

        println!(
            "Fleet Size: {:>4} cars ({:>4} tires) | Frame Latency: {:>8.2?} | {:>8.0} FPS max | {:>5.2}% of 16.6ms budget",
            fleet_size, fleet_size * 4, frame_latency, fps_capacity, percent_60fps
        );
    }

    // 4. Raw Pacejka 'Magic Formula' Equation Micro-Benchmark
    println!("\n--- 4. Raw 6-Parameter Pacejka 'Magic Formula' Curve Evaluation ---");
    {
        let pacejka = Pacejka::default();
        let iterations = 10_000_000;
        let start = Instant::now();
        let mut force_sum = 0.0;

        for i in 0..iterations {
            let slip = (i as f32 * 0.0001).sin() * 0.3;
            force_sum += pacejka.evaluate(slip);
        }

        let elapsed = start.elapsed();
        let ns_per_eval = elapsed.as_nanos() as f64 / iterations as f64;
        let evals_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Pacejka Curve Evals: {} | Time: {:.2?} | Latency: {:.2} ns/eval ({:>10.0} evals/s) | Sum: {:.1}",
            iterations, elapsed, ns_per_eval, evals_per_sec, force_sum
        );
    }

    println!("\n============================================================");
    println!("                      Benchmark Complete                    ");
    println!("============================================================");
}
