/// A struct, resembling a support.
/// The alpha is the alpha to the global coordinate system
/// three values resemble whether its DOF is free and whether it is associated with a feather
/// [true, true, false]
/// [x1  , x2  , phi3 ]
pub struct support {
    alpha: f64,
    is_free: [bool; 3]
    federnach: [f64; 3],
}

impl support {
    
}
