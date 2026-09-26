//! Complete tire force evaluation including camber thrust, self-aligning torque ($M_z$), and pneumatic trail.
//!
//! # Mechanics & Formulations
//!
//! ## Camber Thrust
//! When a wheel is tilted at camber angle $\gamma$ relative to the road surface,
//! a lateral camber thrust force is produced in the direction of the tilt:
//!
//! $$F_{y,\gamma} = C_{\gamma} \cdot \gamma \cdot F_z$$
//!
//! where $C_{\gamma}$ is the camber stiffness coefficient (typically 0.05 to 0.15 rad$^{-1}$)
//! and $F_z$ is the tire normal load.
//!
//! ## Self-Aligning Torque ($M_z$) & Pneumatic Trail ($t$)
//! Because the tire contact patch deforms progressively from front to rear,
//! the centroid of the lateral friction force $F_y$ is located behind the wheel centerline
//! by a distance known as the **pneumatic trail** $t(\alpha)$:
//!
//! $$t(\alpha) = D_t \cdot \cos\left(C_t \cdot \arctan\left(B_t \cdot \alpha - E_t \cdot (B_t \cdot \alpha - \arctan(B_t \cdot \alpha))\right)\right)$$
//!
//! The fundamental self-aligning torque that gives authentic steering wheel feel is:
//!
//! $$M_z = -t(\alpha) \cdot F_y + M_{zr}(\alpha, \gamma)$$
//!
//! - **At small slip angles ($\alpha < 4^\circ$)**: Pneumatic trail $t$ is positive and large,
//!   producing strong self-centering torque that aligns the wheel straight ahead.
//! - **At peak grip ($\alpha \approx 6^\circ - 8^\circ$)**: Self-aligning torque reaches its peak
//!   slightly before lateral force peaks, providing crucial tactile warning of impending slide.
//! - **In a drift ($\alpha > 12^\circ$)**: Pneumatic trail collapses towards zero or negative,
//!   causing the steering to go light and naturally counter-steer into the drift!

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::combined_slip::{CombinedSlipModel, FrictionCircleConfig};
use crate::pacejka::{Pacejka, PacejkaCoefficients};
use crate::slip::{SlipCalculator, SlipState};

/// Configuration for camber stiffness and self-aligning torque.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AligningTorqueConfig {
    /// Camber thrust stiffness coefficient $C_{\gamma}$ (fraction of $F_z$ per radian of camber).
    pub camber_stiffness: f32,
    /// Peak pneumatic trail $D_t$ in meters (typically 0.02 - 0.04 m for passenger/race tires).
    pub peak_trail: f32,
    /// Pneumatic trail Pacejka stiffness factor $B_t$.
    pub trail_b: f32,
    /// Pneumatic trail Pacejka shape factor $C_t$.
    pub trail_c: f32,
    /// Pneumatic trail Pacejka curvature factor $E_t$.
    pub trail_e: f32,
    /// Residual torque factor $D_r$ in N·m / N.
    pub residual_torque_coeff: f32,
}

impl Default for AligningTorqueConfig {
    fn default() -> Self {
        Self {
            camber_stiffness: 0.08,
            peak_trail: 0.028, // 28 mm pneumatic trail
            trail_b: 6.5,
            trail_c: 1.45,
            trail_e: -1.8,
            residual_torque_coeff: 0.002,
        }
    }
}

/// Comprehensive output of the tire dynamics evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TireForcesOutput {
    /// Longitudinal force $F_x$ in Newtons (forward positive).
    pub longitudinal_force: f32,
    /// Pure cornering lateral force before camber in Newtons (right positive).
    pub cornering_force: f32,
    /// Camber thrust force in Newtons.
    pub camber_thrust: f32,
    /// Total net lateral force $F_y$ in Newtons (cornering + camber thrust).
    pub lateral_force: f32,
    /// Self-aligning torque $M_z$ around vertical axis in N·m (positive resists right turn).
    pub aligning_torque: f32,
    /// Pneumatic trail $t(\alpha)$ in meters.
    pub pneumatic_trail: f32,
    /// Combined horizontal force vector $(F_x, F_y)$ in Newtons.
    pub horizontal_force: Vec2,
    /// Normal load $F_z$ in Newtons.
    pub normal_load: f32,
    /// Tire slip kinematics.
    pub slip: SlipState,
    /// Friction circle utilization ratio.
    pub grip_utilization: f32,
    /// Effective friction coefficient $\mu$.
    pub effective_mu: f32,
    /// True if tire is sliding / drifting beyond static grip.
    pub is_sliding: bool,
}

/// Full tire model assembling Pacejka Magic Formula, combined slip, camber thrust, and aligning torque.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TireModel {
    /// Pure longitudinal force Pacejka curve.
    pub longitudinal_pacejka: Pacejka,
    /// Pure lateral force Pacejka curve.
    pub lateral_pacejka: Pacejka,
    /// Aligning torque and camber parameters.
    pub aligning_config: AligningTorqueConfig,
    /// Combined slip and friction circle model.
    pub combined_slip: CombinedSlipModel,
    /// Slip kinematics calculator.
    pub slip_calculator: SlipCalculator,
}

impl Default for TireModel {
    fn default() -> Self {
        Self {
            longitudinal_pacejka: Pacejka::new(PacejkaCoefficients {
                b: 11.0,
                c: 1.65,
                d: 1.0,
                e: -0.5,
                sh: 0.0,
                sv: 0.0,
            }),
            lateral_pacejka: Pacejka::new(PacejkaCoefficients {
                b: 9.5,
                c: 1.40,
                d: 1.0,
                e: -1.2,
                sh: 0.0,
                sv: 0.0,
            }),
            aligning_config: AligningTorqueConfig::default(),
            combined_slip: CombinedSlipModel::new(FrictionCircleConfig::default()),
            slip_calculator: SlipCalculator::default(),
        }
    }
}

impl TireModel {
    /// Creates a custom tire model.
    pub fn new(
        longitudinal_pacejka: Pacejka,
        lateral_pacejka: Pacejka,
        aligning_config: AligningTorqueConfig,
        friction_config: FrictionCircleConfig,
    ) -> Self {
        Self {
            longitudinal_pacejka,
            lateral_pacejka,
            aligning_config,
            combined_slip: CombinedSlipModel::new(friction_config),
            slip_calculator: SlipCalculator::default(),
        }
    }

    /// Evaluates pneumatic trail $t(\alpha)$ in meters:
    ///
    /// $$t(\alpha) = D_t \cdot \cos\left(C_t \cdot \arctan\left(B_t \cdot \alpha - E_t \cdot (B_t \cdot \alpha - \arctan(B_t \cdot \alpha))\right)\right)$$
    pub fn calculate_pneumatic_trail(&self, slip_angle_rad: f32) -> f32 {
        let alpha = slip_angle_rad;
        let bt_alpha = self.aligning_config.trail_b * alpha;
        let inner = bt_alpha - self.aligning_config.trail_e * (bt_alpha - bt_alpha.atan());
        let angle = self.aligning_config.trail_c * inner.atan();
        self.aligning_config.peak_trail * angle.cos()
    }

    /// Evaluates camber thrust force:
    ///
    /// $$F_{y,\gamma} = C_{\gamma} \cdot \gamma \cdot F_z$$
    pub fn calculate_camber_thrust(&self, camber_angle_rad: f32, fz: f32) -> f32 {
        self.aligning_config.camber_stiffness * camber_angle_rad * fz
    }

    /// Evaluates complete tire forces, aligning torque, and slip kinematics.
    ///
    /// # Arguments
    /// - `angular_velocity`: Wheel spin speed $\omega$ in rad/s.
    /// - `wheel_radius`: Tire rolling radius $r$ in meters.
    /// - `longitudinal_velocity`: Contact speed in tire forward direction $v_x$ (m/s).
    /// - `lateral_velocity`: Contact speed in tire sideways direction $v_y$ (m/s).
    /// - `normal_load`: Vertical tire load $F_z$ in Newtons.
    /// - `camber_angle_rad`: Wheel inclination angle $\gamma$ in radians.
    pub fn evaluate(
        &self,
        angular_velocity: f32,
        wheel_radius: f32,
        longitudinal_velocity: f32,
        lateral_velocity: f32,
        normal_load: f32,
        camber_angle_rad: f32,
    ) -> TireForcesOutput {
        if normal_load <= 1.0 {
            return TireForcesOutput {
                longitudinal_force: 0.0,
                cornering_force: 0.0,
                camber_thrust: 0.0,
                lateral_force: 0.0,
                aligning_torque: 0.0,
                pneumatic_trail: 0.0,
                horizontal_force: Vec2::ZERO,
                normal_load: 0.0,
                slip: SlipState::default(),
                grip_utilization: 0.0,
                effective_mu: self.combined_slip.config.peak_friction,
                is_sliding: false,
            };
        }

        // 1. Compute slip kinematics (slip ratio kappa and slip angle alpha)
        let slip = self.slip_calculator.calculate(
            angular_velocity,
            wheel_radius,
            longitudinal_velocity,
            lateral_velocity,
        );

        // 2. Evaluate uncoupled Pacejka friction factors
        let norm_fx0 = self.longitudinal_pacejka.evaluate(slip.slip_ratio);
        let norm_fy0 = self.lateral_pacejka.evaluate(slip.slip_angle_rad);

        let uncoupled_fx = norm_fx0 * normal_load;
        let uncoupled_fy = -norm_fy0 * normal_load; // Opposes lateral slip velocity

        // 3. Combined slip and friction circle coupling
        let combined = self.combined_slip.combine_forces(
            uncoupled_fx,
            uncoupled_fy,
            normal_load,
            slip.slip_ratio,
            slip.slip_angle_rad,
        );

        // 4. Camber thrust
        let camber_thrust = self.calculate_camber_thrust(camber_angle_rad, normal_load);
        let total_fy = combined.fy + camber_thrust;

        // 5. Pneumatic trail and self-aligning torque
        let trail = self.calculate_pneumatic_trail(slip.slip_angle_rad);
        let residual_torque = self.aligning_config.residual_torque_coeff
            * normal_load
            * camber_angle_rad;
        let aligning_torque = -trail * total_fy + residual_torque;

        TireForcesOutput {
            longitudinal_force: combined.fx,
            cornering_force: combined.fy,
            camber_thrust,
            lateral_force: total_fy,
            aligning_torque,
            pneumatic_trail: trail,
            horizontal_force: Vec2::new(combined.fx, total_fy),
            normal_load,
            slip,
            grip_utilization: combined.grip_utilization,
            effective_mu: combined.effective_mu,
            is_sliding: combined.is_sliding,
        }
    }
}
