//! Combined slip interaction, normal load sensitivity, and friction circle / ellipse dynamics.
//!
//! # Mechanics & Formulations
//!
//! ## Normal Load Sensitivity
//! Real pneumatic tires exhibit *load sensitivity*: as normal load $F_z$ increases,
//! the tire's friction coefficient $\mu$ decreases slightly:
//!
//! $$\mu(F_z) = \mu_0 \cdot \left(1 - k_{\text{load}} \cdot \frac{F_z - F_{z0}}{F_{z0}}\right)$$
//!
//! where $F_{z0}$ is the rated nominal design load, $\mu_0$ is the baseline friction coefficient,
//! and $k_{\text{load}}$ is the load sensitivity de-rating factor (typically 0.10 to 0.25).
//!
//! ## Friction Circle & Ellipse Limits
//! The total horizontal force that a tire contact patch can sustain is bounded by the normal force:
//!
//! $$\sqrt{F_x^2 + F_y^2} \le \mu_{\text{peak}} \cdot F_z$$
//!
//! Because tire rubber compounds have different longitudinal and lateral shear moduli,
//! an elliptical envelope is commonly employed:
//!
//! $$\left(\frac{F_x}{F_{x,\text{max}}}\right)^2 + \left(\frac{F_y}{F_{y,\text{max}}}\right)^2 \le 1$$
//!
//! ## Pacejka Combined Slip Weighting Functions
//! Hans Pacejka's combined slip model modifies the pure slip forces ($F_{x0}, F_{y0}$)
//! using interaction weighting functions ($G_{xa}, G_{yk}$):
//!
//! $$G_{xa}(\alpha) = \cos\left(C_{xa} \cdot \arctan(B_{xa} \cdot \alpha)\right)$$
//! $$G_{yk}(\kappa) = \cos\left(C_{yk} \cdot \arctan(B_{yk} \cdot \kappa)\right)$$
//! $$F_x = F_{x0} \cdot G_{xa}(\alpha), \quad F_y = F_{y0} \cdot G_{yk}(\kappa)$$
//!
//! ## Progressive Slide / Drift Regime
//! Once the resultant shear force exceeds the peak grip envelope, the tire enters progressive slide.
//! The friction coefficient smoothly transitions from static peak $\mu_{\text{peak}}$
//! down towards the kinetic sliding coefficient $\mu_{\text{slide}}$:
//!
//! $$\mu_{\text{eff}} = \mu_{\text{slide}} + (\mu_{\text{peak}} - \mu_{\text{slide}}) \cdot \exp\left(-\beta \cdot (\sigma - 1)^2\right)$$
//!
//! providing the progressive breakaway and controllable handling needed for authentic drift physics.

use glam::Vec2;
use serde::{Deserialize, Serialize};

/// Parameters defining tire friction limits, load sensitivity, and drift transition.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FrictionCircleConfig {
    /// Peak static friction coefficient $\mu_{\text{peak}}$ (e.g. 1.15 for high performance sport tires).
    pub peak_friction: f32,
    /// Kinetic sliding friction coefficient $\mu_{\text{slide}}$ during full wheelspin or drift (e.g. 0.78).
    pub slide_friction: f32,
    /// Reference nominal normal load $F_{z0}$ in Newtons (e.g. 3,500 N).
    pub nominal_load: f32,
    /// Load sensitivity factor $k_{\text{load}}$ (rate of friction coefficient loss per unit load).
    pub load_sensitivity: f32,
    /// Combined slip weighting stiffness for longitudinal force under lateral slip ($B_{xa}$).
    pub b_xa: f32,
    /// Combined slip weighting shape factor ($C_{xa}$).
    pub c_xa: f32,
    /// Combined slip weighting stiffness for lateral force under longitudinal slip ($B_{yk}$).
    pub b_yk: f32,
    /// Combined slip weighting shape factor ($C_{yk}$).
    pub c_yk: f32,
    /// Breakaway progression exponent $\beta$ controlling drift feel.
    pub drift_smoothness: f32,
}

impl Default for FrictionCircleConfig {
    fn default() -> Self {
        Self {
            peak_friction: 1.15,
            slide_friction: 0.80,
            nominal_load: 3_500.0,
            load_sensitivity: 0.15,
            b_xa: 8.5,
            c_xa: 1.10,
            b_yk: 7.2,
            c_yk: 1.10,
            drift_smoothness: 4.0,
        }
    }
}

/// Resolved combined slip forces and friction envelope status.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CombinedSlipResult {
    /// Net longitudinal force $F_x$ in Newtons (forward positive).
    pub fx: f32,
    /// Net lateral force $F_y$ in Newtons (right positive).
    pub fy: f32,
    /// Total horizontal vector force $(F_x, F_y)$ in Newtons.
    pub total_force: Vec2,
    /// Current effective friction coefficient $\mu_{\text{eff}}$ accounting for load sensitivity & slide.
    pub effective_mu: f32,
    /// Maximum available horizontal grip $F_{\text{grip\_max}} = \mu_{\text{eff}} \cdot F_z$ in Newtons.
    pub max_available_grip: f32,
    /// Fraction of available friction circle consumed $\sqrt{F_x^2 + F_y^2} / (\mu F_z)$.
    pub grip_utilization: f32,
    /// True if tire forces exceed peak static grip (sliding / drifting).
    pub is_sliding: bool,
}

/// Evaluator for combined tire slip forces and friction circle constraints.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CombinedSlipModel {
    /// Configuration constants.
    pub config: FrictionCircleConfig,
}

impl CombinedSlipModel {
    /// Creates a new combined slip model.
    pub const fn new(config: FrictionCircleConfig) -> Self {
        Self { config }
    }

    /// Computes load-sensitive peak friction coefficient for normal load $F_z$:
    ///
    /// $$\mu(F_z) = \mu_0 \cdot \left(1 - k_{\text{load}} \cdot \frac{F_z - F_{z0}}{F_{z0}}\right)$$
    pub fn load_sensitive_mu(&self, fz: f32) -> f32 {
        if fz <= 1.0 {
            return self.config.peak_friction;
        }

        let delta_load = (fz - self.config.nominal_load) / self.config.nominal_load;
        let factor = (1.0 - self.config.load_sensitivity * delta_load).clamp(0.5, 1.4);
        (self.config.peak_friction * factor).max(0.2)
    }

    /// Computes Pacejka longitudinal interaction multiplier $G_{xa}(\alpha)$:
    ///
    /// $$G_{xa}(\alpha) = \cos\left(C_{xa} \cdot \arctan(B_{xa} \cdot \alpha)\right)$$
    pub fn g_xa(&self, slip_angle_rad: f32) -> f32 {
        let angle = self.config.c_xa * (self.config.b_xa * slip_angle_rad).atan();
        angle.cos().clamp(0.0, 1.0)
    }

    /// Computes Pacejka lateral interaction multiplier $G_{yk}(\kappa)$:
    ///
    /// $$G_{yk}(\kappa) = \cos\left(C_{yk} \cdot \arctan(B_{yk} \cdot \kappa)\right)$$
    pub fn g_yk(&self, slip_ratio: f32) -> f32 {
        let angle = self.config.c_yk * (self.config.b_yk * slip_ratio).atan();
        angle.cos().clamp(0.0, 1.0)
    }

    /// Combines pure longitudinal force ($F_{x0}$) and pure lateral force ($F_{y0}$)
    /// subject to friction circle saturation, load sensitivity, and progressive slide.
    ///
    /// # Arguments
    /// - `fx0`: Pure longitudinal force from uncoupled Pacejka model (N).
    /// - `fy0`: Pure lateral force from uncoupled Pacejka model (N).
    /// - `fz`: Current tire normal load (N).
    /// - `slip_ratio`: Longitudinal slip ratio $\kappa$.
    /// - `slip_angle_rad`: Lateral slip angle $\alpha$ in radians.
    pub fn combine_forces(
        &self,
        fx0: f32,
        fy0: f32,
        fz: f32,
        slip_ratio: f32,
        slip_angle_rad: f32,
    ) -> CombinedSlipResult {
        if fz <= 1.0 {
            return CombinedSlipResult {
                fx: 0.0,
                fy: 0.0,
                total_force: Vec2::ZERO,
                effective_mu: self.config.peak_friction,
                max_available_grip: 0.0,
                grip_utilization: 0.0,
                is_sliding: false,
            };
        }

        // Apply Pacejka combined slip coupling factors
        let gxa = self.g_xa(slip_angle_rad);
        let gyk = self.g_yk(slip_ratio);

        let initial_fx = fx0 * gxa;
        let initial_fy = fy0 * gyk;

        let requested_force = Vec2::new(initial_fx, initial_fy);
        let requested_mag = requested_force.length();

        // Load-adjusted peak friction
        let peak_mu = self.load_sensitive_mu(fz);
        let max_peak_force = peak_mu * fz;

        let utilization = if max_peak_force > 1.0 {
            requested_mag / max_peak_force
        } else {
            0.0
        };

        let is_sliding = utilization > 1.0;

        // Progressive slide transition
        let (effective_mu, final_force) = if is_sliding {
            let over_ratio = utilization - 1.0;
            let decay = (-self.config.drift_smoothness * over_ratio * over_ratio).exp();
            let eff_mu = self.config.slide_friction + (peak_mu - self.config.slide_friction) * decay;
            let clamped_mag = eff_mu * fz;
            let clamped_force = requested_force.normalize_or_zero() * clamped_mag;
            (eff_mu, clamped_force)
        } else {
            (peak_mu, requested_force)
        };

        CombinedSlipResult {
            fx: final_force.x,
            fy: final_force.y,
            total_force: final_force,
            effective_mu,
            max_available_grip: effective_mu * fz,
            grip_utilization: utilization,
            is_sliding,
        }
    }
}
