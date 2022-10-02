use libm::atan2;
use libm::cos;
use libm::sin;
use nalgebra::DMatrix;
use nalgebra::Dynamic;
use nalgebra::Matrix;
use nalgebra::OMatrix;
use nalgebra::SMatrix;
use nalgebra::{DVector, SVector};
use std::f64::consts;

use crate::stiffness::system::*;

type Matrix3x3 = SMatrix<f64, 3, 3>;
type Matrix6x6 = SMatrix<f64, 6, 6>;
type Vector6 = SVector<f64, 6>;
type MatrixDxD = OMatrix<f64, Dynamic, Dynamic>;
type VectorD = DVector<f64>;

impl Beam {
    fn local_boundary_forces(
        &self,
        lenght: f64,
        lineload: Option<StaticLinearLineload>,
        local_vector: Vector6,
    ) {
        let (stiff, load_vec) = self.local_stiffness_and_load(lenght, lineload);

        let mut rsk = stiff * local_vector + load_vec;
        {
            //TM Definitionen
            rsk[0] = rsk[0]; // Druck ist irgendwie komisch
            rsk[1] = -rsk[1];
            rsk[2] = rsk[2];
            rsk[3] = -rsk[3]; // Druck ist komisch
            rsk[4] = rsk[4];
            rsk[5] = -rsk[5];
        }
        let r = BeamResult::new(&rsk.as_slice());

        println!("{:?}", rsk.map(|f| ((f * 1000.0).round() / 1000.0)));
    }
    fn local_stiffness_and_load(
        &self,
        lenght: f64,
        lineload: Option<StaticLinearLineload>,
    ) -> (Matrix6x6, Vector6) {
        let lineload = match lineload {
            None => StaticLinearLineload::new_constant_load(0.0),
            Some(t) => t,
        };
        let mut resVec = Vector6::new(
            0.0,
            -lenght / 20.0
                * (7.0 * lineload.get_from_perpendicular_load()
                    + 3.0 * lineload.get_to_perpendicular_load()),
            -lenght * lenght / 60.0
                * (3.0 * lineload.get_from_perpendicular_load()
                    + 2.0 * lineload.get_to_perpendicular_load()),
            0.0,
            -lenght / 20.0
                * (3.0 * lineload.get_from_perpendicular_load()
                    + 7.0 * lineload.get_to_perpendicular_load()),
            lenght * lenght / 60.0
                * (2.0 * lineload.get_from_perpendicular_load()
                    + 3.0 * lineload.get_to_perpendicular_load()),
        );

        let ei = self.get_emodul() * self.get_ftm();
        let ea = self.get_emodul() * self.get_area();

        let mut resMat = Matrix6x6::new(
            ea / lenght,
            0.0,
            0.0,
            -ea / lenght,
            0.0,
            0.0,
            0.0, // 2te Zeile
            (12.0 * ei) / (lenght.powi(3)),
            (6.0 * ei) / (lenght.powi(2)),
            0.0,
            -(12.0 * ei) / (lenght.powi(3)),
            (6.0 * ei) / (lenght.powi(2)),
            0.0, // 3te Zeile
            (6.0 * ei) / (lenght.powi(2)),
            (4.0 * ei) / (lenght),
            0.0,
            -(6.0 * ei) / (lenght.powi(2)),
            (2.0 * ei) / (lenght),
            -ea / lenght, // 4te Zeile
            0.0,
            0.0,
            ea / lenght,
            0.0,
            0.0,
            0.0, // 5te Zeile
            -(12.0 * ei) / (lenght.powi(3)),
            -(6.0 * ei) / (lenght.powi(2)),
            0.0,
            (12.0 * ei) / (lenght.powi(3)),
            -(6.0 * ei) / (lenght.powi(2)),
            0.0, // 6te Zeile
            (6.0 * ei) / (lenght.powi(2)),
            (2.0 * ei) / (lenght),
            0.0,
            -(6.0 * ei) / (lenght.powi(2)),
            (4.0 * ei) / (lenght),
        );
        for i in 0..self.get_dofs().len() {
            if self.get_dofs()[i] {
                let talpha = if i < 3 {
                    self.get_start_alpha()
                } else {
                    self.get_end_alpha()
                };
                // Transformieren in das KOS der Unstetigkeit
                resMat = transmatrix6x6(talpha).transpose() * resMat * transmatrix6x6(talpha); // FIXME eventuell tauschen
                resVec = transmatrix6x6(talpha).transpose() * resVec;
                let m = resMat.row(i);
                let k = resMat[(i, i)];
                let f = resVec[i];
                let uh = 1.0 / (k + self.get_dofstiffness()[i]);

                let ku = uh * m.tr_mul(&m);
                let fu = uh * f * m;
                resMat = resMat - ku;
                resVec = resVec - fu.transpose();
                resVec = transmatrix6x6(talpha) * resVec;
                resMat = transmatrix6x6(talpha) * resMat * transmatrix6x6(talpha).transpose();
            }
        }
        // Stablokal
        return (resMat, resVec);
    }
}

fn transmatrix3x3(alpha: f64) -> Matrix3x3 {
    Matrix3x3::new(
        cos(alpha),
        sin(alpha),
        0.0,
        -sin(alpha),
        cos(alpha),
        0.0,
        0.0,
        0.0,
        1.0,
    )
}

fn transmatrix6x6(alpha: f64) -> Matrix6x6 {
    Matrix6x6::new(
        cos(alpha),
        sin(alpha),
        0.0,
        0.0,
        0.0,
        0.0, //
        -sin(alpha),
        cos(alpha),
        0.0,
        0.0,
        0.0,
        0.0, //
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0, //
        0.0,
        0.0,
        0.0,
        cos(alpha),
        sin(alpha),
        0.0, //
        0.0,
        0.0,
        0.0,
        -sin(alpha),
        cos(alpha),
        0.0, //
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )
}

impl System {
    pub fn global_stiffness_matrix(&self, loading: &SystemLoading) {
        let ps = self.get_points();

        let total_dofs = ps.len() * 3;
        let mut steif = MatrixDxD::zeros(total_dofs, total_dofs);
        let mut last = VectorD::zeros(total_dofs);

        // Iterieren durch alle Stäbe
        for i in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(i);
            let to = self.get_beam_to_point(i);
            let length = self.get_beam_lenght(i);
            let alpha = self.get_beam_alpha(i);
            println!("{:?}", alpha / consts::PI * 180.0);

            let lineloading = loading.get_total_lineload_for_beam(i);

            let b = &self.get_beams()[i];
            let loc = b.local_stiffness_and_load(length, Some(lineloading));
            let stiff = loc.0;
            let load_vec = loc.1;
            let f = transmatrix6x6(alpha).transpose() * stiff * transmatrix6x6(alpha);
            let lv = transmatrix6x6(alpha).transpose() * load_vec;
            // Assemblierung der Globalen Stabsteifigkeitsmatrix
            for i in 0..3 {
                for j in 0..3 {
                    steif[(from * 3 + i, from * 3 + j)] =
                        steif[(from * 3 + i, from * 3 + j)] + f[(i, j)];
                    steif[(from * 3 + i, to * 3 + j)] =
                        steif[(from * 3 + i, to * 3 + j)] + f[(i, j + 3)];
                    steif[(to * 3 + i, from * 3 + j)] =
                        steif[(to * 3 + i, from * 3 + j)] + f[(i + 3, j)];
                    steif[(to * 3 + i, to * 3 + j)] =
                        steif[(to * 3 + i, to * 3 + j)] + f[(i + 3, j + 3)];
                }
                last[from * 3 + i] = last[from * 3 + i] - lv[i];
                last[to * 3 + i] = last[to * 3 + i] - lv[i + 3];
            }
        }
        // Einarbeiten der Knotenlasten
        for i in 0..loading.get_static_loads().len() {
            let p = loading.get_static_load_points()[i];
            let l = &loading.get_static_loads()[i];

            last[p * 3] = last[p * 3] + l.get_loading()[0];
            last[p * 3 + 1] = last[p * 3 + 1] + l.get_loading()[1];
            last[p * 3 + 2] = last[p * 3 + 2] + l.get_loading()[2];
        }
        // Einarbeiten der Homogenen und TODO Inhomogenen Randbedingungen
        for i in 0..self.get_supports().len() {
            let sup = &self.get_supports()[i];
            let sup_point = self.get_support_points()[i];
            for j in 0..3 {
                if !sup.get_free_dofs()[j] {
                    // kein freier Knoten
                    for k in 0..total_dofs {
                        steif[(k, sup_point * 3 + j)] = 0.0;
                        steif[(sup_point * 3 + j, k)] = 0.0;
                    }
                    steif[(sup_point * 3 + j, sup_point * 3 + j)] = 1.0;
                    last[sup_point * 3 + j] = 0.0;
                }
            }
        }

        println!("{:?}", steif);

        let g = match steif.cholesky() {
            Some(t) => t,
            None => panic!("Matrix nicht positiv definit."),
        };

        // Lösung in Globalen KOS
        let result = g.solve(&last);

        // Lösung der Stäbe
        for i in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(i);
            let to = self.get_beam_to_point(i);
            let length = self.get_beam_lenght(i);
            let alpha = self.get_beam_alpha(i);

            let mut v = Vector6::zeros();
            for j in 0..3 {
                v[j] = result[from * 3 + j];
                v[j + 3] = result[to * 3 + j];
            }
            let lineloading = loading.get_total_lineload_for_beam(i);

            let b = &self.get_beams()[i];

            let v = transmatrix6x6(alpha) * v;
            b.local_boundary_forces(length, Some(lineloading), v);
        }

        println!("{:?}", last);
        println!("{:?}", result);
    }
}

pub struct BeamResult {
    rsk: [f64; 6],
}

impl BeamResult {
    pub fn new(vals: &[f64]) -> Self {
        BeamResult {
            rsk: [vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]],
        }
    }
}
