use fuzzy::fuzzy::*;
use std::f64::consts::PI;

fn main() {
    let f = FuzzyTriangular::new(0.0, 3.0, 6.0);

    for i in 0..=100 {
        let alpha = 0.01 * i as f64;
        let b = alpha_level_optimize(&f, alpha, |x| 2.71_f64.powf(-x));
        println!("a = {} --> [{}, {}]", alpha, b.0, b.1);
    }
}
