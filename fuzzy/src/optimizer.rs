pub fn optimize_direct<F>(lower_bound: f64, upper_bound: f64, func: &F)
where
    F: Fn(f64) -> f64,
{
    let m = 1;
    let a = upper_bound;
    let b = lower_bound;
    let c = (upper_bound + lower_bound) * 0.5;
    let mut fc = func(c);
    let t = 0;
    let mut intervals = Vec::new();
    intervals.push((a, b, c));
    loop {
        let i = intervals.pop().unwrap();
        let delta = (i.1 - i.0) / 3.0;
        let c1 = i.2 - delta;
        let a1 = i.0;
        let b1 = i.0 + delta;
        fc = fc.min(func(c1));

        intervals.push((a1, b1, c1));

        let c2 = i.2 + delta;
        let a2 = i.0 + 2.0 * delta;
        let b2 = i.1;
        fc = fc.min(func(c2));

        intervals.push((a2, b2, c2));
    }
} // ternary heap for better caching...

// 3 oder mehr startpunkte

pub fn optimize_multivariat<F>(lower_bounds: Vec<f64>, upper_bounds: Vec<f64>, func: &F)
where
    F: Fn(Vec<f64>) -> f64,
{
    // Zuerst müssen die grenzen geprüft werden:
    let n = lower_bounds.len();
    for i in 0..n {
        if lower_bounds[i] >= upper_bounds[i] {
            panic!("Upper Bounds are not strictly larger than lower bounds in all coordinates!")
        }
    }
    //
    let mut ncall = 0;
    let mut ncloc = 0;
    //
    // erstellen einer Liste
    let mut l = vec![1, n]; // Liste der Dimensionen
    let mut L = vec![2, n]; // Liste der endpunkte oder der anzahl der partionen entlang der i-ten dimension.

    // Initialisierung start!
    // Initialisierungsliste
    let mut x0 = Vec::new();

    x0.push(lower_bounds.clone());
    x0.push(
        (lower_bounds)
            .iter()
            .zip(upper_bounds.iter())
            .map(|t| (t.0 + t.1) / 2.0)
            .collect(),
    );
    x0.push(upper_bounds.clone());

    //
}
