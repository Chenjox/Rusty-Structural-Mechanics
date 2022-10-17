use crate::stiffness::direct_stiffness::*;
use crate::stiffness::system::*;
use nalgebra::Dynamic;
use nalgebra::OMatrix;
use nalgebra::SMatrix;
use nalgebra::{DVector, SVector};

use crate::stiffness::direct_stiffness::BeamResult;

type Matrix3x3 = SMatrix<f64, 3, 3>;
type Matrix6x6 = SMatrix<f64, 6, 6>;
type Matrix7x7 = SMatrix<f64, 7, 7>;
type Vector7 = SVector<f64, 7>;
type Vector6 = SVector<f64, 6>;
type MatrixDxD = OMatrix<f64, Dynamic, Dynamic>;
type VectorD = DVector<f64>;

impl Beam {
    fn local_mech_boundary_forces_first_order(
        &self,
        length: f64,
        lineload: Option<StaticLinearLineload>,
        local_vector: Vector6,
    ) -> BeamResult {
        let (stiff, load_vec) = self.local_stiffness_and_load_first_order(length, lineload);

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

    fn local_stiffness_and_load_first_order(
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
            let loc = b.local_stiffness_and_load_first_order(length, Some(lineloading));
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
            r.push(b.local_mech_boundary_forces_first_order(length, Some(lineloading), v));
        }

        return BeamResultSet::new(r);
    }
}
