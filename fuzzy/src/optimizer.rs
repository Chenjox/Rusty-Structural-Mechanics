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

pub fn optimize_multivariat<F>(lower_bounds: Vec<f64>, upper_bounds: Vec<f64>, func: &F)
where F: Fn(Vec<f64>) -> f64
{
    
}
