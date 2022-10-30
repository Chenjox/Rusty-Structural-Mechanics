use fuzzy::fuzzy::*;
use gui::util::*;
use std::f64::consts::PI;

fn main() {
    let f = FuzzyTrapezoidal::new(0.0, 4.0, 5.0, 6.0);

    let mut res = String::new();
    for i in 0..=100 {
        let alpha = 0.01 * i as f64;
        let b = alpha_level_optimize(&f, alpha, |x| x * x - 2.0 * x);
        res.push_str(&format!(
            "{},{},{},{},{}\n",
            alpha,
            b.0,
            b.1,
            &f.alpha_level_support(alpha).0,
            &f.alpha_level_support(alpha).1
        ));
    }
    write_file("__test.csv", &res);
}
