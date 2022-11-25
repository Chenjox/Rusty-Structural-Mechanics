use fuzzy::optimization::*;

fn rose(vc: &[f64]) -> f64 {
    let mut r = 0.0;
    for i in 0..vc.len() - 1 {
        r += 100.0 * (vc[i + 1] - (vc[i].powi(2))).powi(2) + (1.0 - vc[i]).powi(2);
    }
    return r;
}

pub fn main() {
    let lower = vec![-2.0, -2.0];
    let upper = vec![2.0, 2.0];

    let start = vec![0.0, 0.0];
    let r = simplex_optimization(lower, upper, start, rose);
    println!("{:?}", r);
}