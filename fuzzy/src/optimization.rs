fn add(a: f64, av: &Vec<f64>, b: f64, bv: &Vec<f64>, erg: &mut Vec<f64>) {
    for i in 0..erg.len() {
        erg[i] = a * av[i] + b * bv[i]
    }
}

fn simplex_centroid(vals: &[Vec<f64>], excluded: usize, n: usize) -> Vec<f64> {
    let mut g = vec![0.0; n];
    for j in 0..n {
        for i in 0..n + 1 {
            if i != excluded {
                g[j] = g[j] + vals[i][j];
            }
        }
        g[j] = 1.0 / ((n) as f64) * g[j];
    }
    return g;
}

fn is_in_bounds(
    lower_bound: &Vec<f64>,
    upper_bound: &Vec<f64>,
    value: &Vec<f64>,
    n: usize,
) -> bool {
    let mut in_bound = true;
    for j in 0..n {
        in_bound = in_bound && lower_bound[j] < value[j] && upper_bound[j] > value[j];
    }
    return in_bound;
}

fn reflection(center: &Vec<f64>, tobereflected: &Vec<f64>, n: usize) -> Vec<f64> {
    let mut g = vec![0.0; n];
    for j in 0..n {
        g[j] = center[j] + 1.0 * (center[j] - tobereflected[j]);
    }
    return g;
}

fn expand(center: &Vec<f64>, tobe: &Vec<f64>, n: usize) -> Vec<f64> {
    let mut g = vec![0.0; n];
    for j in 0..n {
        g[j] = center[j] + 2.0 * (tobe[j] - center[j]);
    }
    return g;
}

fn contract(center: &Vec<f64>, tobe: &Vec<f64>, n: usize) -> Vec<f64> {
    let mut g = vec![0.0; n];
    for j in 0..n {
        g[j] = center[j] + 0.5 * (tobe[j] - center[j]);
    }
    return g;
}

fn sort_vals(points: &mut Vec<Vec<f64>>, point_values: &mut Vec<f64>) {
    let mut i = 1;
    while i < point_values.len() {
        let x = point_values[i];
        let px = points[i].clone();
        let mut j = i - 1;
        let mut zero_signal = false;
        'inner: while point_values[j] > x {
            if j == 0 {
                point_values[j + 1] = point_values[j];
                points[j + 1] = points[j].clone();
                zero_signal = true;
                break 'inner;
            } else {
                point_values[j + 1] = point_values[j];
                points[j + 1] = points[j].clone();
                j = j - 1;
            }
        }
        if zero_signal {
            point_values[0] = x;
            points[0] = px;
        } else {
            point_values[j + 1] = x;
            points[j + 1] = px;
        }

        i = i + 1;
    }
}

pub fn simplex_optimization<F>(
    lower_bound: Vec<f64>,
    upper_bound: Vec<f64>,
    start_point: Vec<f64>,
    func: F,
) -> Vec<f64>
where
    F: Fn(&Vec<f64>) -> f64,
{
    let n = lower_bound.len();

    let mut points = Vec::with_capacity(n + 1);
    // Anfangspunkte
    {
        // Werden mittels lerp erstellt.
        for i in 0..n {
            // nehme einen Schritt im Bereich
            let interval = (start_point[i] + lower_bound[i]) * 0.5;
            // erstelle einen neuen punkt
            let mut new_point = vec![0.0; n];
            for k in 0..n {
                // der in der Koordinate k vom startpunkt abweicht
                if k == i {
                    new_point[k] = start_point[k] + interval * 0.7
                } else {
                    // und sonst gleich ist.
                    new_point[k] = start_point[k]
                }
            }
            // und füge Ihn zur der point liste hinzu
            points.push(new_point);
        }
    }
    // letztlich der startpunkt
    points.push(Vec::from(start_point));
    // erste funktionswerte
    let mut point_values = vec![0.0; n + 1];
    for i in 0..n + 1 {
        point_values[i] = func(&points[i]);
    }
    // sortieren der Punkte
    let mut iter_counter = 0;
    let MAX_ITER = 10000;
    'itera: loop {
        if iter_counter > MAX_ITER {
            break 'itera;
        }
        iter_counter += 1;
        sort_vals(&mut points, &mut point_values);
        // hier kommt die terminierung hin.
        {
            let mut dist = 0.0;
            for k in 0..n {
                dist += (points[0][k] - points[n][k]).powi(2);
            }
            dist = dist.sqrt();
            if dist < 1e-10 {
                println!("{}", iter_counter);
                break 'itera;
            }
        }

        // vorne steht das beste, hinten das schlimmste
        // Schritt 2
        let o = simplex_centroid(&points, n, n);

        // Schritt 3
        let r = reflection(&o, &points[n], n);
        let fr = if is_in_bounds(&lower_bound, &upper_bound, &r, n) {
            func(&r)
        } else {
            point_values[n] + 1.0
        };

        if point_values[0] <= fr && fr < point_values[n - 1] {
            points[n] = r;
            point_values[n] = fr;
            continue 'itera;
        }
        // Schritt 4.
        if fr < point_values[0] {
            // reflected is best!
            let e = expand(&o, &r, n);
            let fe = if is_in_bounds(&lower_bound, &upper_bound, &e, n) {
                func(&e)
            } else {
                point_values[n] + 1.0
            };
            if fe < fr {
                points[n] = e;
                point_values[n] = fe;
            } else {
                points[n] = r;
                point_values[n] = fr;
            }
            continue 'itera;
        }
        // Schritt 5
        if fr < point_values[n] {
            let c = contract(&o, &r, n);
            let fc = if is_in_bounds(&lower_bound, &upper_bound, &c, n) {
                func(&c)
            } else {
                point_values[n] + 1.0
            };
            if fc < fr {
                points[n] = c;
                point_values[n] = fc;
                continue 'itera;
            } else {
                // schrinking
                for i in 1..n + 1 {
                    for k in 0..n {
                        points[i][k] = points[0][k] + 0.5 * (points[i][k] - points[0][k]);
                    }
                }
                for i in 1..n + 1 {
                    point_values[i] = func(&points[i]);
                }
                continue 'itera;
            }
        }
        if fr >= point_values[n] {
            let c = contract(&o, &points[n], n); // Dieser Punkt muss innerhalb liegen!
            let fc = func(&c);
            if fc < point_values[n] {
                points[n] = c;
                point_values[n] = fc;
                continue 'itera;
            } else {
                // schrinking
                for i in 1..n + 1 {
                    for k in 0..n {
                        points[i][k] = points[0][k] + 0.5 * (points[i][k] - points[0][k]);
                    }
                }
                for i in 1..n + 1 {
                    point_values[i] = func(&points[i]);
                }
                continue 'itera;
            }
        }
    }
    return points[0].clone();
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    const EPS: f64 = 1e-8_f64;

    #[test]
    fn rosenbrocks_function() {
        fn rosenbrock(vc: &Vec<f64>) -> f64 {
            let mut r = 0.0;
            for i in 0..vc.len() - 1 {
                r += 100.0 * (vc[i + 1] - (vc[i].powi(2))).powi(2) + (1.0 - vc[i]).powi(2);
            }
            return r;
        }
        let lower = vec![-2.0, -2.0];
        let upper = vec![2.0, 2.0];

        let start = vec![0.0, 0.0];
        let r = simplex_optimization(lower, upper, start, rosenbrock);
        assert_approx_eq!(r[0], 1.0, EPS);
        assert_approx_eq!(r[1], 1.0, EPS);
    }

    #[test]
    fn double_linear_function() {
        fn optimum_corner(vc: &Vec<f64>) -> f64 {
            return vc[0] + vc[1];
        }
        let lower = vec![-2.0, -2.0];
        let upper = vec![2.0, 2.0];

        let start = vec![0.0, 0.0];
        let r = simplex_optimization(lower, upper, start, optimum_corner);
        assert_approx_eq!(r[0], -2.0, EPS);
        assert_approx_eq!(r[1], -2.0, EPS);
    }
}
