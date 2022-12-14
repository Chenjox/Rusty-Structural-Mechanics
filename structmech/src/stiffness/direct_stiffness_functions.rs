fn func_f1_p(omega: f64) -> f64 {
    if omega < 0.1 {
        return 1.0 - 1.0 / 10.0 * omega.powi(2);
    } else {
        let denom = 12.0 * (2.0 * (1.0 - cos(omega)) - omega * sin(omega));
        let numer = omega.powi(3) * sin(omega);
        return numer / denom;
    }
}

fn func_f2_p(omega: f64) -> f64 {
    if omega < 0.1 {
        return 1.0 - 1.0 / 60.0 * omega.powi(2);
    } else {
        let denom = 6.0 * (2.0 * (1.0 - cos(omega)) - omega * sin(omega));
        let numer = omega.powi(2) * (1.0 - cos(omega));
        return numer / denom;
    }
}

fn func_f3_p(omega: f64) -> f64 {
    if omega < 0.1 {
        return 1.0 - 1.0 / 30.0 * omega.powi(2);
    } else {
        let denom = 4.0 * (2.0 * (1.0 - cos(omega)) - omega * sin(omega));
        let numer = omega * (sin(omega) - omega * cos(omega));
        return numer / denom;
    }
}

fn func_f4_p(omega: f64) -> f64 {
    if omega < 0.1 {
        return 1.0 + 1.0 / 60.0 * omega.powi(2);
    } else {
        let denom = 2.0 * (2.0 * (1.0 - cos(omega)) - omega * sin(omega));
        let numer = omega * (omega - sin(omega));
        return numer / denom;
    }
}

pub(crate) fn transmatrix3x3(alpha: f64) -> Matrix3x3 {
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

pub(crate) fn transmatrix6x6(alpha: f64) -> Matrix6x6 {
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
