//! Tire slip kinematics: longitudinal slip ratio ($\kappa$) and lateral slip angle ($\alpha$).
//!
//! # Mathematical Formulations
//!
//! ## Longitudinal Slip Ratio ($\kappa$)
//! Longitudinal slip measures the relative difference between the tire surface tangential speed
//! ($\omega \cdot r$) and the wheel hub forward speed ($v_x$):
//!
//! $$\kappa = \frac{\omega \cdot r - v_x}{|v_x|}$$
//!
//! - **Driving / Acceleration**: $\omega \cdot r > v_x \implies \kappa > 0$ (wheel spins faster than vehicle).
//!   At full burnout: $\kappa \to +\infty$ (clamped).
//! - **Braking**: $\omega \cdot r < v_x \implies \kappa < 0$.
//!   At complete wheel lock ($ \omega = 0 $): $\kappa = -1.0$.
//! - **Zero-Speed Regularization**:
//!   As forward velocity approaches zero ($|v_x| \to 0$), direct division causes numerical blowup.
//!   We apply a smooth continuous regularization:
//!   $$\kappa = \frac{\omega \cdot r - v_x}{\max(|v_x|, v_{\text{threshold}})}$$
//!
//! ## Lateral Slip Angle ($\alpha$)
//! The slip angle is the angle between the tire heading direction (wheel plane)
//! and its actual velocity vector:
//!
//! $$\alpha = \arctan\left(\frac{v_y}{|v_x|}\right)$$
//!
//! where $v_x$ is the longitudinal contact velocity and $v_y$ is the lateral contact velocity.
//! When $v_x = 0$, the angle cleanly saturates to $\operatorname{sgn}(v_y) \cdot \frac{\pi}{2}$ ($\pm 90^\circ$).

use std::f32::consts::FRAC_PI_2;
use serde::{Deserialize, Serialize};

/// Comprehensive kinematic slip state of a tire contact patch.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SlipState {
    /// Signed longitudinal slip ratio $\kappa \in [-1.0, \dots]$.
    /// Negative indicates braking; positive indicates drive slip/acceleration.
    pub slip_ratio: f32,
    /// Absolute longitudinal slip ratio $|\kappa|$.
    pub abs_slip_ratio: f32,
    /// Lateral slip angle $\alpha$ in radians.
    pub slip_angle_rad: f32,
    /// Lateral slip angle $\alpha$ in degrees.
    pub slip_angle_deg: f32,
    /// Absolute lateral slip angle $|\alpha|$ in degrees.
    pub abs_slip_angle_deg: f32,
    /// Euclidean magnitude of the combined slip vector $\sqrt{\kappa^2 + \tan^2(\alpha)}$.
    pub combined_slip_magnitude: f32,
}

impl Default for SlipState {
    fn default() -> Self {
        Self {
            slip_ratio: 0.0,
            abs_slip_ratio: 0.0,
            slip_angle_rad: 0.0,
            slip_angle_deg: 0.0,
            abs_slip_angle_deg: 0.0,
            combined_slip_magnitude: 0.0,
        }
    }
}

/// Evaluator for tire slip ratio and slip angle.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SlipCalculator {
    /// Low-speed velocity threshold in m/s below which slip calculations are regularized.
    pub low_speed_threshold: f32,
    /// Maximum clamp limit for longitudinal slip ratio (e.g. 2.5).
    pub max_slip_ratio: f32,
}

impl Default for SlipCalculator {
    fn default() -> Self {
        Self {
            low_speed_threshold: 0.40, // 0.4 m/s (~1.4 km/h)
            max_slip_ratio: 3.0,
        }
    }
}

impl SlipCalculator {
    /// Creates a new slip calculator with specified low-speed threshold.
    pub const fn new(low_speed_threshold: f32, max_slip_ratio: f32) -> Self {
        Self {
            low_speed_threshold,
            max_slip_ratio,
        }
    }

    /// Computes the signed longitudinal slip ratio:
    ///
    /// $$\kappa = \frac{\omega \cdot r - v_x}{\max(|v_x|, v_{\text{threshold}})}$$
    ///
    /// # Arguments
    /// - `angular_velocity`: Wheel spin rate $\omega$ in rad/s.
    /// - `wheel_radius`: Tire rolling radius $r$ in meters.
    /// - `longitudinal_velocity`: Contact patch forward speed $v_x$ in m/s.
    pub fn calculate_slip_ratio(
        &self,
        angular_velocity: f32,
        wheel_radius: f32,
        longitudinal_velocity: f32,
    ) -> f32 {
        let wheel_linear_speed = angular_velocity * wheel_radius;
        let delta_v = wheel_linear_speed - longitudinal_velocity;

        let abs_vx = longitudinal_velocity.abs();

        if abs_vx < self.low_speed_threshold {
            // Smooth blend towards zero slip at full stop to avoid jitter
            let speed_fraction = abs_vx / self.low_speed_threshold;
            let raw_slip = delta_v / self.low_speed_threshold;
            (raw_slip * speed_fraction).clamp(-self.max_slip_ratio, self.max_slip_ratio)
        } else {
            let raw_slip = delta_v / abs_vx;
            raw_slip.clamp(-self.max_slip_ratio, self.max_slip_ratio)
        }
    }

    /// Computes the lateral slip angle in radians:
    ///
    /// $$\alpha = \arctan\left(\frac{v_y}{|v_x|}\right)$$
    ///
    /// # Arguments
    /// - `lateral_velocity`: Contact patch sideways speed $v_y$ in m/s (positive = right).
    /// - `longitudinal_velocity`: Contact patch forward speed $v_x$ in m/s.
    pub fn calculate_slip_angle_rad(
        &self,
        lateral_velocity: f32,
        longitudinal_velocity: f32,
    ) -> f32 {
        let abs_vx = longitudinal_velocity.abs();

        if abs_vx < 1e-4 {
            if lateral_velocity.abs() < 1e-4 {
                0.0
            } else {
                lateral_velocity.signum() * FRAC_PI_2
            }
        } else {
            (lateral_velocity / abs_vx).atan()
        }
    }

    /// Computes complete SlipState combining longitudinal and lateral slips.
    pub fn calculate(
        &self,
        angular_velocity: f32,
        wheel_radius: f32,
        longitudinal_velocity: f32,
        lateral_velocity: f32,
    ) -> SlipState {
        let slip_ratio = self.calculate_slip_ratio(angular_velocity, wheel_radius, longitudinal_velocity);
        let slip_angle_rad = self.calculate_slip_angle_rad(lateral_velocity, longitudinal_velocity);
        let slip_angle_deg = slip_angle_rad.to_degrees();

        let tan_alpha = slip_angle_rad.tan();
        let combined_slip_magnitude = (slip_ratio * slip_ratio + tan_alpha * tan_alpha).sqrt();

        SlipState {
            slip_ratio,
            abs_slip_ratio: slip_ratio.abs(),
            slip_angle_rad,
            slip_angle_deg,
            abs_slip_angle_deg: slip_angle_deg.abs(),
            combined_slip_magnitude,
        }
    }
}
