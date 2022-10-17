pub mod direct_stiffness;

/// A Solver for Beam-Truss Systems using first order theory.
///
/// It exposes the method `direct_stiffness_method_first_order` to a `System` which will calculate the internal forces
/// using a specific loading.
pub mod first_order;
pub mod system;
