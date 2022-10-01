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

impl SystemLoading {
    pub fn new(
        loaded_points: Vec<usize>,
        staticloads: Vec<StaticLoad>,
        loaded_beams: Vec<usize>,
        lineloads: Vec<StaticLinearLineload>,
    ) -> Self {
        return SystemLoading {
            loaded_points,
            staticloads,
            loaded_beams,
            lineloads,
        };
    }

    pub fn get_static_loads(&self) -> &[StaticLoad] {
        &self.staticloads
    }
    pub fn get_static_load_points(&self) -> &[usize] {
        &self.loaded_points
    }

    pub fn get_total_lineload_for_beam(&self, beamindex: usize) -> StaticLinearLineload {
        let mut res = StaticLinearLineload::new_constant_load(0.0);
        for i in 0..self.loaded_beams.len() {
            if self.loaded_beams[i] == beamindex {
                res.add_mut(&self.lineloads[i])
            }
        }
        return res;
    }
}

pub struct StaticLoad {
    loading: [f64; 3], // x1 x2 phi3
}
impl StaticLoad {
    pub fn new(global_x: f64, global_y: f64, moment: f64) -> Self {
        StaticLoad {
            loading: [global_x, global_y, moment],
        }
    }
    pub fn get_loading(&self) -> [f64; 3] {
        self.loading
    }
}

pub struct StaticLinearLineload {
    loading: [f64; 4], // Start, Ende
}

impl StaticLinearLineload {
    pub fn new_constant_load(value: f64) -> Self {
        StaticLinearLineload {
            loading: [0.0, 0.0, value, value],
        }
    }
    pub fn get_from_perpendicular_load(&self) -> f64 {
        self.loading[2]
    }
    pub fn get_to_perpendicular_load(&self) -> f64 {
        self.loading[3]
    }
    pub fn add(&self, other: StaticLinearLineload) -> Self {
        StaticLinearLineload {
            loading: [
                self.loading[0] + other.loading[0],
                self.loading[1] + other.loading[1],
                self.loading[2] + other.loading[2],
                self.loading[3] + other.loading[3],
            ],
        }
    }

    pub fn add_mut(&mut self, other: &StaticLinearLineload) {
        self.loading[0] += other.loading[0];
        self.loading[1] += other.loading[1];
        self.loading[2] += other.loading[2];
        self.loading[3] += other.loading[3];
    }
}
