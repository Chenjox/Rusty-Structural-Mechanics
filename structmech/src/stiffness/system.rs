/// A struct, resembling a support.
/// The alpha is the alpha to the global coordinate system
/// three values resemble whether its DOF is free and whether it is associated with a feather
/// [true, true, false]
/// [x1  , x2  , phi3 ]
pub struct Support {
    alpha: f64,
    is_free: [bool; 3],
    federnach: [f64; 3],
}

impl Support {
    pub fn new(alpha: f64, is_free: [bool; 3], federnach: [f64; 3]) -> Self {
        Support {
            alpha,
            is_free,
            federnach,
        }
    }
    pub fn get_alpha(&self) -> f64 {
        self.alpha
    }
}

pub struct Point {
    x: f64,
    y: f64,
}

pub struct Crosssection {
    emodul: f64,
    area: f64,
    ftm: f64,
}

/// x_1, x_2, phi_3 -- x_1, x_2, phi_3
pub struct Beam {
    crosssection: Crosssection,
    dof: [bool; 6],
    dofstiffness: [f64; 6],
    start_dof_alpha: f64,
    end_dof_alpha: f64,
}

impl Beam {
    pub fn get_emodul(&self) -> f64 {
        self.crosssection.emodul
    }
    pub fn get_area(&self) -> f64 {
        self.crosssection.area
    }
    pub fn get_ftm(&self) -> f64 {
        self.crosssection.ftm
    }
    pub fn get_dofs(&self) -> &[bool] {
        &self.dof
    }
    pub fn get_dofstiffness(&self) -> &[f64] {
        &self.dofstiffness
    }
}

pub struct System {
    points: Vec<Point>,
    beam_points: Vec<[usize; 2]>,
    beams: Vec<Beam>,
    support_points: Vec<usize>,
    supports: Vec<Support>,
}

pub struct SystemLoading {
    loading_points: Vec<usize>,
    staticloads: Vec<StaticLoad>,
    loaded_beams: Vec<usize>,
    lineloads: Vec<StaticLinearLineload>,
}

pub struct StaticLoad {
    loading: [f64; 3], // x1 x2 phi3
}

pub struct StaticLinearLineload {
    loading: [f64; 4], // Start, Ende
}
