//! # TLab-Vehicle-Physics-rs
//!
//! Pure Rust Pacejka 'Magic Formula' tire friction, combined slip, and drift physics
//! translated from TLabAltoh's [`TLabVehiclePhysics`](https://github.com/TLabAltoh/TLabVehiclePhysics).
//!
//! Designed for Bevy and pure Rust vehicle simulations requiring realistic tire modeling,
//! progressive drift dynamics, camber thrust, and steering self-aligning torque ($M_z$).
//!
//! ## Subsystems
//! - **Pacejka 'Magic Formula' (`pacejka`)**: Full 6-parameter model ($B, C, D, E, S_h, S_v$)
//!   and multi-curve table interpolation (`MultiPacejka`).
//! - **Slip Kinematics (`slip`)**: Continuous longitudinal slip ratio ($\kappa$) with low-speed
//!   regularization and lateral slip angle ($\alpha$) in degrees and radians.
//! - **Combined Slip & Friction Circle (`combined_slip`)**: Normal load sensitivity $F_z$,
//!   friction circle / ellipse constraints, Pacejka interaction weighting ($G_{xa}, G_{yk}$),
//!   and progressive slide / drift breakaway transition.
//! - **Tire Forces & Alignment (`tire_forces`)**: Camber thrust, pneumatic trail $t(\alpha)$,
//!   and self-aligning torque ($M_z$) for realistic steering force feedback.

pub mod combined_slip;
pub mod pacejka;
pub mod slip;
pub mod tire_forces;

pub use combined_slip::{CombinedSlipModel, CombinedSlipResult, FrictionCircleConfig};
pub use pacejka::{MultiPacejka, MultiPacejkaElement, Pacejka, PacejkaCoefficients};
pub use slip::{SlipCalculator, SlipState};
pub use tire_forces::{AligningTorqueConfig, TireForcesOutput, TireModel};

/// Common prelude types for tlab-vehicle-physics.
pub mod prelude {
    pub use crate::combined_slip::{CombinedSlipModel, CombinedSlipResult, FrictionCircleConfig};
    pub use crate::pacejka::{MultiPacejka, MultiPacejkaElement, Pacejka, PacejkaCoefficients};
    pub use crate::slip::{SlipCalculator, SlipState};
    pub use crate::tire_forces::{AligningTorqueConfig, TireForcesOutput, TireModel};
}
