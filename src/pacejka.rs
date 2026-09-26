//! Full Pacejka 'Magic Formula' tire model and multi-curve interpolation.
//!
//! # Mathematical Formulation
//! The Hans B. Pacejka "Magic Formula" is the industry standard semi-empirical tire model
//! relating slip quantities (slip ratio $\kappa$, slip angle $\alpha$) to tire forces:
//!
//! $$y(x) = D \cdot \sin\left(C \cdot \arctan\left(B \cdot x_1 - E \cdot (B \cdot x_1 - \arctan(B \cdot x_1))\right)\right) + S_v$$
//! where:
//! - $x_1 = x + S_h$ (input slip with horizontal shift)
//! - $B$: **Stiffness Factor** (determines initial slope $\left.\frac{dy}{dx}\right|_{0} = B \cdot C \cdot D$)
//! - $C$: **Shape Factor** (determines limits of sine function, typically 1.3..1.9)
//! - $D$: **Peak Factor** (determines peak force or maximum friction coefficient $\mu$)
//! - $E$: **Curvature Factor** (controls transition and post-peak drop to sliding friction)
//! - $S_h$: **Horizontal Shift** (offsets origin due to ply-steer, conicity, or camber)
//! - $S_v$: **Vertical Shift** (offsets force due to residual lateral force or rolling resistance)
//!
//! ## TLabAltoh Formulation
//! TLabVehiclePhysics uses the core formulation:
//! $$F(x) = D \cdot \sin(C \cdot \arctan(B \cdot x - E \cdot \arctan(B \cdot x - \arctan(B \cdot x))))$$
//! as well as standard Pacejka formulations and piecewise-linear table interpolation (`MultiPacejka`).

use serde::{Deserialize, Serialize};

/// Coefficients defining a Pacejka Magic Formula curve.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PacejkaCoefficients {
    /// Stiffness factor $B$. Determines initial slope together with $C$ and $D$.
    pub b: f32,
    /// Shape factor $C$. Controls asymptotic curve shape (typically 1.3 - 1.9).
    pub c: f32,
    /// Peak factor $D$. Maximum value of the curve before shifts.
    pub d: f32,
    /// Curvature factor $E$. Controls post-peak curve profile and drop-off.
    pub e: f32,
    /// Horizontal shift $S_h$.
    pub sh: f32,
    /// Vertical shift $S_v$.
    pub sv: f32,
}

impl Default for PacejkaCoefficients {
    fn default() -> Self {
        Self {
            b: 10.0,
            c: 1.65,
            d: 1.0,
            e: -1.0,
            sh: 0.0,
            sv: 0.0,
        }
    }
}

/// Pacejka Magic Formula evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pacejka {
    /// Curve coefficients.
    pub coeffs: PacejkaCoefficients,
}

impl Pacejka {
    /// Creates a new Pacejka evaluator with given coefficients.
    pub const fn new(coeffs: PacejkaCoefficients) -> Self {
        Self { coeffs }
    }

    /// Creates a standard 4-parameter Pacejka model without shifts ($S_h = 0, S_v = 0$).
    pub const fn with_bcde(b: f32, c: f32, d: f32, e: f32) -> Self {
        Self {
            coeffs: PacejkaCoefficients {
                b,
                c,
                d,
                e,
                sh: 0.0,
                sv: 0.0,
            },
        }
    }

    /// Evaluates the full Magic Formula:
    ///
    /// $$y(x) = D \cdot \sin(C \cdot \arctan(B \cdot x_1 - E \cdot (B \cdot x_1 - \arctan(B \cdot x_1)))) + S_v$$
    /// where $x_1 = x + S_h$.
    pub fn evaluate(&self, x: f32) -> f32 {
        let x1 = x + self.coeffs.sh;
        let bx = self.coeffs.b * x1;
        let inner = bx - self.coeffs.e * (bx - bx.atan());
        self.coeffs.d * (self.coeffs.c * inner.atan()).sin() + self.coeffs.sv
    }

    /// Evaluates using the exact C# formula from TLabAltoh/TLabVehiclePhysics:
    ///
    /// $$F(x) = D \cdot \sin(C \cdot \arctan(B \cdot x - E \cdot \arctan(B \cdot x - \arctan(B \cdot x))))$$
    pub fn evaluate_tlab(&self, x: f32) -> f32 {
        let bx = self.coeffs.b * x;
        let inner = bx - self.coeffs.e * (bx - bx.atan()).atan();
        self.coeffs.d * (self.coeffs.c * inner.atan()).sin()
    }

    /// Evaluates the initial stiffness at zero slip:
    ///
    /// $$\left.\frac{dy}{dx}\right|_{x=0} = B \cdot C \cdot D$$
    #[inline]
    pub fn cornering_stiffness(&self) -> f32 {
        self.coeffs.b * self.coeffs.c * self.coeffs.d
    }

    /// Linearly interpolates between two Pacejka curves.
    pub fn lerp(p0: &Pacejka, p1: &Pacejka, factor: f32) -> Self {
        let f = factor.clamp(0.0, 1.0);
        let lerp_val = |a: f32, b: f32| a + (b - a) * f;
        Self {
            coeffs: PacejkaCoefficients {
                b: lerp_val(p0.coeffs.b, p1.coeffs.b),
                c: lerp_val(p0.coeffs.c, p1.coeffs.c),
                d: lerp_val(p0.coeffs.d, p1.coeffs.d),
                e: lerp_val(p0.coeffs.e, p1.coeffs.e),
                sh: lerp_val(p0.coeffs.sh, p1.coeffs.sh),
                sv: lerp_val(p0.coeffs.sv, p1.coeffs.sv),
            },
        }
    }
}

/// An entry in a multi-curve Pacejka lookup table indexed by a primary variable (e.g. normal load $F_z$ or slip).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MultiPacejkaElement {
    /// Table index value (must be monotonically increasing).
    pub index: f32,
    /// Associated Pacejka curve at this index.
    pub pacejka: Pacejka,
}

/// Multi-curve Pacejka container performing bilinear or piecewise-linear interpolation
/// across varying operating conditions (e.g. load dependency, surface conditions).
///
/// Ported from `TLab.VehiclePhysics.PacejkaTool.MultiPacejka`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MultiPacejka {
    /// Table entries sorted by `index`.
    pub table: Vec<MultiPacejkaElement>,
}

impl MultiPacejka {
    /// Creates a new MultiPacejka table.
    ///
    /// Entries are automatically sorted by index.
    pub fn new(mut elements: Vec<MultiPacejkaElement>) -> Self {
        elements.sort_by(|a, b| a.index.partial_cmp(&b.index).unwrap_or(std::cmp::Ordering::Equal));
        Self { table: elements }
    }

    /// Finds the bounding Pacejka curves and interpolation factor for input $x$.
    pub fn get_bounding_curves(&self, x: f32) -> (Pacejka, Pacejka, f32) {
        if self.table.is_empty() {
            let default_p = Pacejka::default();
            return (default_p, default_p, 0.0);
        }

        if self.table.len() == 1 || x <= self.table[0].index {
            return (self.table[0].pacejka, self.table[0].pacejka, 0.0);
        }

        let last_idx = self.table.len() - 1;
        if x >= self.table[last_idx].index {
            return (self.table[last_idx].pacejka, self.table[last_idx].pacejka, 1.0);
        }

        // Binary search for interval
        let mut low = 0;
        let mut high = last_idx;
        while low < high - 1 {
            let mid = (low + high) / 2;
            if self.table[mid].index <= x {
                low = mid;
            } else {
                high = mid;
            }
        }

        let idx0 = low;
        let idx1 = low + 1;
        let denom = (self.table[idx1].index - self.table[idx0].index).max(1e-6);
        let factor = ((x - self.table[idx0].index) / denom).clamp(0.0, 1.0);

        (self.table[idx0].pacejka, self.table[idx1].pacejka, factor)
    }

    /// Evaluates the 2D interpolated Magic Formula.
    ///
    /// # Arguments
    /// - `x`: Primary indexing variable (e.g. normal force $F_z$ in N, or slip angle in deg).
    /// - `y`: Secondary evaluation variable (e.g. slip ratio $\kappa$, or slip angle $\alpha$).
    pub fn evaluate(&self, x: f32, y: f32) -> f32 {
        let (p0, p1, factor) = self.get_bounding_curves(x);
        let val0 = p0.evaluate(y);
        let val1 = p1.evaluate(y);
        val0 * (1.0 - factor) + val1 * factor
    }
}

impl Default for Pacejka {
    fn default() -> Self {
        Self::new(PacejkaCoefficients::default())
    }
}
