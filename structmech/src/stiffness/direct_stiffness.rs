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

include!("direct_stiffness_functions.rs");

impl Beam {
    fn local_mech_boundary_forces(
        &self,
        lenght: f64,
        lineload: Option<StaticLinearLineload>,
        local_vector: Vector6,
    ) -> BeamResult {
        let (stiff, load_vec) = self.local_stiffness_and_load(lenght, lineload);

        let mut rsk = stiff * local_vector + load_vec;
        {
            //TM Definitionen
            rsk[0] = -rsk[0]; // Druck ist irgendwie komisch
            rsk[1] = -rsk[1];
            rsk[2] = rsk[2];
            rsk[3] = rsk[3]; // Druck ist komisch
            rsk[4] = rsk[4];
            rsk[5] = -rsk[5];
        }
        let r = BeamResult::new(&rsk.as_slice(), &local_vector.as_slice());

        //println!("{:?}", rsk.map(|f| ((f * 1000.0).round() / 1000.0)));
        return r;
    }
    fn local_mech_boundary_forces_second_order(
        &self,
        lenght: f64,
        lineload: Option<StaticLinearLineload>,
        local_vector: Vector6,
        normal_component: f64,
    ) -> BeamResult {
        let (stiff, load_vec) =
            self.local_stiffness_and_load_second_order(lenght, normal_component, lineload);

        let mut rsk = stiff * local_vector + load_vec;
        {
            //TM Definitionen
            rsk[0] = -rsk[0]; // Druck ist irgendwie komisch
            rsk[1] = -rsk[1];
            rsk[2] = rsk[2];
            rsk[3] = rsk[3]; // Druck ist komisch
            rsk[4] = rsk[4];
            rsk[5] = -rsk[5];
        }
        let r = BeamResult::new(&rsk.as_slice(), &local_vector.as_slice());

        //println!("{:?}", rsk.map(|f| ((f * 1000.0).round() / 1000.0)));
        return r;
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
        let (resVec, resMat) = self.local_discontinuity(lenght, resVec, resMat);
        // Stablokal
        return (resMat, resVec);
    }

    fn local_stiffness_and_load_second_order(
        &self,
        lenght: f64,
        normal_force: f64,
        lineload: Option<StaticLinearLineload>,
    ) -> (Matrix6x6, Vector6) {
        let lineload = match lineload {
            None => StaticLinearLineload::new_constant_load(0.0),
            Some(t) => t,
        };
        let ei = self.get_emodul() * self.get_ftm();
        let ea = self.get_emodul() * self.get_area();

        let omega = if normal_force < 0.0 {
            lenght * (-normal_force / ei).sqrt()
        } else {
            0.0
        };

        let f1 = func_f1_p(omega);
        let f2 = func_f2_p(omega);
        let f3 = func_f3_p(omega);
        let f4 = func_f4_p(omega);

        let e_1 = (30.0 / omega.powi(2)) * (f4 / 3.0 - f3 / 6.0);
        let e_2 = (20.0 / omega.powi(2)) * (1.0 + f4 / 6.0 - f3 / 3.0);

        //println!("e_1 = {}", e_1);
        //println!("e_2 = {}", e_2);

        let resVec = Vector6::new(
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

        let resMat = Matrix6x6::new(
            ea / lenght,
            0.0,
            0.0,
            -ea / lenght,
            0.0,
            0.0,
            0.0, // 2te Zeile
            (12.0 * ei) / (lenght.powi(3)) * f1,
            (6.0 * ei) / (lenght.powi(2)) * f2,
            0.0,
            -(12.0 * ei) / (lenght.powi(3)) * f1,
            (6.0 * ei) / (lenght.powi(2)) * f2,
            0.0, // 3te Zeile
            (6.0 * ei) / (lenght.powi(2)) * f2,
            (4.0 * ei) / (lenght) * f3,
            0.0,
            -(6.0 * ei) / (lenght.powi(2)) * f2,
            (2.0 * ei) / (lenght) * f4,
            -ea / lenght, // 4te Zeile
            0.0,
            0.0,
            ea / lenght,
            0.0,
            0.0,
            0.0, // 5te Zeile
            -(12.0 * ei) / (lenght.powi(3)) * f1,
            -(6.0 * ei) / (lenght.powi(2)) * f2,
            0.0,
            (12.0 * ei) / (lenght.powi(3)) * f1,
            -(6.0 * ei) / (lenght.powi(2)) * f2,
            0.0, // 6te Zeile
            (6.0 * ei) / (lenght.powi(2)) * f2,
            (2.0 * ei) / (lenght) * f4,
            0.0,
            -(6.0 * ei) / (lenght.powi(2)) * f2,
            (4.0 * ei) / (lenght) * f3,
        );
        let (resVec, resMat) = self.local_discontinuity(lenght, resVec, resMat);
        // Stablokal
        return (resMat, resVec);
    }

    fn local_discontinuity(
        &self,
        lenght: f64,
        mut res_vec: Vector6,
        mut res_mat: Matrix6x6,
    ) -> (Vector6, Matrix6x6) {
        for i in 0..self.get_dofs().len() {
            if self.get_dofs()[i] {
                let talpha = if i < 3 {
                    self.get_start_alpha()
                } else {
                    self.get_end_alpha()
                };
                // Transformieren in das KOS der Unstetigkeit
                res_mat = transmatrix6x6(talpha).transpose() * (res_mat) * transmatrix6x6(talpha); // FIXME eventuell tauschen
                res_vec = transmatrix6x6(talpha).transpose() * (res_vec);
                let m = res_mat.row(i);
                let k = res_mat[(i, i)];
                let f = res_vec[i];
                let uh = 1.0 / (k + self.get_dofstiffness()[i]);

                let ku = uh * m.tr_mul(&m);
                let fu = uh * f * m;
                res_mat = res_mat - ku;
                res_vec = res_vec - fu.transpose();
                res_vec = transmatrix6x6(talpha) * res_vec;
                res_mat = transmatrix6x6(talpha) * res_mat * transmatrix6x6(talpha).transpose();
            }
        }
        return (res_vec, res_mat);
    }
}

impl Support {
    pub fn stiffness_matrix(&self) -> Matrix3x3 {
        let m = Matrix3x3::new(
            if self.get_free_dofs()[0] {
                self.get_feder()[0]
            } else {
                0.0
            },
            0.0,
            0.0,
            0.0,
            if self.get_free_dofs()[1] {
                self.get_feder()[1]
            } else {
                0.0
            },
            0.0,
            0.0,
            0.0,
            if self.get_free_dofs()[2] {
                self.get_feder()[2]
            } else {
                0.0
            },
        );
        let m = transmatrix3x3(self.get_alpha()) * m;
        // Global
        return m;
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
    fn stiffness_matrix_first_order(
        &self,
        loading: &SystemLoading,
        steif: &mut MatrixDxD,
        last: &mut VectorD,
    ) {
        for i in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(i);
            let to = self.get_beam_to_point(i);
            let length = self.get_beam_lenght(i);
            let alpha = self.get_beam_alpha(i);

            let lineloading = loading.get_total_lineload_for_beam(i);

            let b = &self.get_beams()[i];
            let loc = b.local_stiffness_and_load(length, Some(lineloading));
            let stiff = loc.0;
            let load_vec = loc.1;
            let f = transmatrix6x6(alpha) * stiff * transmatrix6x6(alpha).transpose();
            let lv = transmatrix6x6(alpha) * load_vec;
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
    }

    fn knotenlasten(loading: &SystemLoading, last: &mut VectorD) {
        for i in 0..loading.get_static_loads().len() {
            let p = loading.get_static_load_points()[i];
            let l = &loading.get_static_loads()[i];

            last[p * 3] = last[p * 3] + l.get_loading()[0];
            last[p * 3 + 1] = last[p * 3 + 1] + l.get_loading()[1];
            last[p * 3 + 2] = last[p * 3 + 2] + l.get_loading()[2];
        }
    }

    fn supports(&self, dofs: usize, steif: &mut MatrixDxD, last: &mut VectorD) {
        // Einarbeiten der Homogenen und TODO Inhomogenen Randbedingungen
        for i in 0..self.get_supports().len() {
            let sup = &self.get_supports()[i];
            let sup_point = self.get_support_points()[i];

            let m = sup.stiffness_matrix();

            for i in 0..3 {
                for j in 0..3 {
                    steif[(sup_point * 3 + i, sup_point * 3 + j)] =
                        steif[(sup_point * 3 + i, sup_point * 3 + j)] + m[(i, j)]
                }
            }

            for j in 0..3 {
                if !sup.get_free_dofs()[j] {
                    // TODO Gedrehte Supports
                    // kein freier Knoten
                    for k in 0..dofs {
                        steif[(k, sup_point * 3 + j)] = 0.0;
                        steif[(sup_point * 3 + j, k)] = 0.0;
                    }
                    steif[(sup_point * 3 + j, sup_point * 3 + j)] = 1.0;
                    last[sup_point * 3 + j] = 0.0;
                }
            }
        }
    }

    pub fn matrix_stiffness_method_first_order(&self, loading: &SystemLoading) -> BeamResultSet {
        let ps = self.get_points();

        let total_dofs = ps.len() * 3;
        let mut steif = MatrixDxD::zeros(total_dofs, total_dofs);
        let mut last = VectorD::zeros(total_dofs);

        // Iterieren durch alle Stäbe
        self.stiffness_matrix_first_order(loading, &mut steif, &mut last);

        // Einarbeiten der Knotenlasten
        System::knotenlasten(loading, &mut last);

        self.supports(total_dofs, &mut steif, &mut last);

        let g = match steif.cholesky() {
            Some(t) => t,
            None => panic!("Matrix nicht positiv definit."),
        };

        // Lösung in Globalen KOS
        let result = g.solve(&last);

        // Lösung der Stäbe
        let mut r = Vec::new();
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

            let v = transmatrix6x6(alpha).transpose() * v;
            r.push(b.local_mech_boundary_forces(length, Some(lineloading), v));
        }

        return BeamResultSet::new(r);
    }

    pub fn matrix_stiffness_method_second_order(&self, loading: &SystemLoading) -> BeamResultSet {
        let ps = self.get_points();

        let total_dofs = ps.len() * 3;
        let mut steif = MatrixDxD::zeros(total_dofs, total_dofs);
        let mut last = VectorD::zeros(total_dofs);
        // Erste Iteration
        let first_iter = self.matrix_stiffness_method_first_order(loading);

        // Iterieren durch alle Stäbe
        for i in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(i);
            let to = self.get_beam_to_point(i);
            let length = self.get_beam_lenght(i);
            let alpha = self.get_beam_alpha(i);
            let normal_component = 0.5 * (first_iter.res[i].rsk[0] + first_iter.res[i].rsk[3]);

            let lineloading = loading.get_total_lineload_for_beam(i);

            let b = &self.get_beams()[i];
            let loc = b.local_stiffness_and_load_second_order(
                length,
                normal_component,
                Some(lineloading),
            );
            let stiff = loc.0;
            let load_vec = loc.1;
            let f = transmatrix6x6(alpha) * stiff * transmatrix6x6(alpha).transpose();
            let lv = transmatrix6x6(alpha) * load_vec;
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
        System::knotenlasten(loading, &mut last);

        self.supports(total_dofs, &mut steif, &mut last);

        let g = match steif.cholesky() {
            Some(t) => t,
            None => panic!("Matrix nicht positiv definit."),
        };

        // Lösung in Globalen KOS
        let result = g.solve(&last);

        // Lösung der Stäbe
        let mut r = Vec::new();
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

            let normal_component = 0.5 * (first_iter.res[i].rsk[0] + first_iter.res[i].rsk[3]);

            let b = &self.get_beams()[i];

            let v = transmatrix6x6(alpha).transpose() * v;
            r.push(b.local_mech_boundary_forces_second_order(
                length,
                Some(lineloading),
                v,
                normal_component,
            ));
        }

        return BeamResultSet::new(r);
    }

    pub fn matrix_stiffness_method_second_order_matrix(
        &self,
        loading: &SystemLoading,
    ) -> MatrixDxD {
        let ps = self.get_points();

        let total_dofs = ps.len() * 3;
        let mut steif = MatrixDxD::zeros(total_dofs, total_dofs);
        let mut last = VectorD::zeros(total_dofs);
        // Erste Iteration
        let first_iter = self.matrix_stiffness_method_first_order(loading);

        // Iterieren durch alle Stäbe
        for i in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(i);
            let to = self.get_beam_to_point(i);
            let length = self.get_beam_lenght(i);
            let alpha = self.get_beam_alpha(i);
            let normal_component = 0.5 * (first_iter.res[i].rsk[0] + first_iter.res[i].rsk[3]);

            let lineloading = loading.get_total_lineload_for_beam(i);

            let b = &self.get_beams()[i];
            let loc = b.local_stiffness_and_load_second_order(
                length,
                normal_component,
                Some(lineloading),
            );
            let stiff = loc.0;
            let load_vec = loc.1;
            let f = transmatrix6x6(alpha) * stiff * transmatrix6x6(alpha).transpose();
            let lv = transmatrix6x6(alpha) * load_vec;
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
        System::knotenlasten(loading, &mut last);

        self.supports(total_dofs, &mut steif, &mut last);

        return steif;
    }
}

pub struct BeamResultSet {
    res: Vec<BeamResult>,
}

impl BeamResultSet {
    pub fn new(res: Vec<BeamResult>) -> Self {
        BeamResultSet { res }
    }
    pub fn get_results(&self) -> &[BeamResult] {
        return &self.res;
    }
}

pub struct BeamResult {
    rsk: [f64; 6],
    rv: [f64; 6],
}

impl BeamResult {
    pub fn new(rsk: &[f64], rv: &[f64]) -> Self {
        BeamResult {
            rsk: [rsk[0], rsk[1], rsk[2], rsk[3], rsk[4], rsk[5]],
            rv: [rv[0], rv[1], rv[2], rv[3], rv[4], rv[5]],
        }
    }
    pub fn get_rsks(&self) -> &[f64; 6] {
        return &self.rsk;
    }
}

// TODO impl System result mit reduktionsmethode...
// TODO hinzufügen von Einzellasten im Stab.
