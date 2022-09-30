use libm::atan2;
use libm::cos;
use libm::sin;
use nalgebra::DMatrix;
use nalgebra::Dynamic;
use nalgebra::Matrix;
use nalgebra::OMatrix;
use nalgebra::SMatrix;
use nalgebra::{DVector, SVector};

use crate::stiffness::system::*;

type Matrix3x3 = SMatrix<f64, 3, 3>;
type Matrix6x6 = SMatrix<f64, 6, 6>;
type Vector6 = SVector<f64, 6>;
type MatrixDxD = OMatrix<f64, Dynamic, Dynamic>;
type VectorD = DVector<f64>;

impl Beam {
    fn local_stiffness(&self, lenght: f64) -> Matrix6x6 {
        let ei = self.get_emodul() * self.get_ftm();
        let ea = self.get_emodul() * self.get_area();

        let mut res = Matrix6x6::new(
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
        // TODO Unstetigkeiten in einem KOS
        for i in 0..self.get_dofs().len() {
            if self.get_dofs()[i] {
                let talpha = if i < 3 {
                    self.get_start_alpha()
                } else {
                    self.get_end_alpha()
                };
                res = transmatrix6x6(talpha) * res * transmatrix6x6(talpha).transpose();
                let m = res.row(i);
                let k = res[(i, i)];
                let uh = 1.0 / (k + self.get_dofstiffness()[i]);

                let ku = uh * m.tr_mul(&m);
                res = res - ku;
                res = transmatrix6x6(talpha).transpose() * res * transmatrix6x6(talpha);
            }
        }
        // Stablokal
        return res;
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
    pub fn global_stiffness_matrix(&self) -> MatrixDxD {
        let ps = self.get_points();
        let sup = self.get_supports();

        let mut total_dofs = ps.len() * 3;
        let mut steif = MatrixDxD::zeros(total_dofs, total_dofs);

        // Iterieren durch alle Stäbe
        for i in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(i);
            let to = self.get_beam_to_point(i);
            let length = self.get_beam_lenght(i);
            let alpha = self.get_beam_alpha(i);

            let b = &self.get_beams()[i];
            let f = transmatrix6x6(self.get_beam_alpha(i)).transpose()
                * b.local_stiffness(length)
                * transmatrix6x6(self.get_beam_alpha(i));
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
            }
        }
        // Einarbeiten der Homogenen und Inhomogenen Randbedingungen
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
                } else {
                    // TODO
                }
            }
        }
        return steif;
    }
}
