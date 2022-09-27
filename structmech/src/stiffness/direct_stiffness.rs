use nalgebra::DMatrix;
use nalgebra::SMatrix;

use crate::stiffness::system::*;

type Matrix3x3 = SMatrix<f64, 3, 3>;
type Matrix6x6 = SMatrix<f64, 6, 6>;

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
                let m = res.row(i);
                let k = res[(i, i)];
                let uh = 1.0 / (k + self.get_dofstiffness()[i]);

                let ku = uh * m.tr_mul(&m);
                res = res - ku;
            }
        }
        // Stablokal
        return res;
    }
}
