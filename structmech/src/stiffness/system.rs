use libm::atan2;

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
    pub fn get_free_dofs(&self) -> &[bool; 3] {
        &self.is_free
    }
    pub fn get_feder(&self) -> &[f64; 3] {
        &self.federnach
    }
}

#[derive(Copy, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }
}

#[derive(Clone, Copy)]
pub struct Crosssection {
    emodul: f64,
    area: f64,
    ftm: f64,
}

impl Crosssection {
    pub fn new(emodul: f64, area: f64, ftm: f64) -> Self {
        Crosssection { emodul, area, ftm }
    }
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
    pub fn new(
        cross: Crosssection,
        dof: [bool; 6],
        dofstiffness: [f64; 6],
        start: f64,
        end: f64,
    ) -> Beam {
        Beam {
            crosssection: cross,
            dof: dof,
            dofstiffness: dofstiffness,
            start_dof_alpha: start,
            end_dof_alpha: end,
        }
    }
    pub fn get_emodul(&self) -> f64 {
        self.crosssection.emodul
    }
    pub fn get_area(&self) -> f64 {
        self.crosssection.area
    }
    pub fn get_ftm(&self) -> f64 {
        self.crosssection.ftm
    }
    pub fn get_start_alpha(&self) -> f64 {
        self.start_dof_alpha
    }
    pub fn get_end_alpha(&self) -> f64 {
        self.end_dof_alpha
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

impl System {
    pub fn new(
        points: Vec<Point>,
        beam_points: Vec<[usize; 2]>,
        beams: Vec<Beam>,
        support_points: Vec<usize>,
        supports: Vec<Support>,
    ) -> Self {
        System {
            points,
            beam_points,
            beams,
            support_points,
            supports,
        }
    }
    pub fn get_points(&self) -> &[Point] {
        return &self.points;
    }
    pub fn get_beam_points(&self) -> &[[usize; 2]] {
        return &self.beam_points;
    }
    pub fn get_beams(&self) -> &[Beam] {
        return &self.beams;
    }
    pub fn get_support_points(&self) -> &[usize] {
        return &self.support_points;
    }
    pub fn get_supports(&self) -> &[Support] {
        return &self.supports;
    }
    pub fn get_beam_lenght(&self, beamindex: usize) -> f64 {
        let p = self.beam_points[beamindex];
        let point_one = &self.points[p[0]];
        let point_two = &self.points[p[1]];
        return ((point_one.x - point_two.x).powi(2) + (point_one.y - point_two.y).powi(2)).sqrt();
    }
    pub fn get_beam_alpha(&self, beamindex: usize) -> f64 {
        let p = self.beam_points[beamindex];
        let point_one = &self.points[p[0]];
        let point_two = &self.points[p[1]];
        return atan2(point_one.y - point_two.y, point_one.x - point_two.x);
    }
    pub fn get_beam_from_point(&self, beamindex: usize) -> usize {
        return self.beam_points[beamindex][0];
    }
    pub fn get_beam_to_point(&self, beamindex: usize) -> usize {
        return self.beam_points[beamindex][1];
    }
}

pub struct SystemLoading {
    loaded_points: Vec<usize>,
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
