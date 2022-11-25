use fuzzy::fuzzy::*;

fn mx0(q: f64, l: f64, x: f64) -> f64 {
    q * 0.5 * (l * l - 2.0 * l * x + x * x)
}

fn mx1(l: f64, x: f64) -> f64 {
    l - x
}

fn lerp(start: f64, end: f64, l: f64, x: f64) -> f64 {
    (start) * (1.0 - x / l) + end * x / l
}

fn delta11(l: f64, emodul: f64, b: f64, h1: f64, h2: f64) -> f64 {
    let partitions = 100;
    let dx = l / (partitions as f64);
    let a = 0.0;
    let mut funcsum = mx1(l, a).powi(2) / (emodul * lerp(h1, h2, l, a).powi(3) * b * 1.0 / 12.0);
    for i in 0..partitions {
        let ahi = a + dx * ((i + 1) as f64);

        funcsum += mx1(l, ahi).powi(2) / (emodul * lerp(h1, h2, l, ahi).powi(3) * b * 1.0 / 12.0);
    }
    let integral = funcsum * dx;
    return integral;
}

fn delta10(q: f64, l: f64, emodul: f64, b: f64, h1: f64, h2: f64) -> f64 {
    let partitions = 100;
    let dx = l / (partitions as f64);
    let a = 0.0;
    let mut funcsum =
        mx1(l, a) * mx0(q, l, a) / (emodul * lerp(h1, h2, l, a).powi(3) * b * 1.0 / 12.0);
    for i in 0..partitions {
        let ahi = a + dx * ((i + 1) as f64);

        funcsum +=
            mx1(l, ahi) * mx0(q, l, ahi) / (emodul * lerp(h1, h2, l, ahi).powi(3) * b * 1.0 / 12.0);
    }
    let integral = funcsum * dx;
    return integral;
}

fn schnittmoment(q: f64, l: f64, emodul: f64, b: f64, h1: f64, h2: f64, k: f64, x: f64) -> f64 {
    let delta10 = delta10(q, l, emodul, b, h1, h2);
    let delta11 = delta11(l, emodul, b, h1, h2);

    let x1 = -(delta10 / (delta11 + 1.0 / k));

    return mx0(q, l, x) + x1 * mx1(l, x);
}

fn get_min_schnittmoment(q: f64, l: f64, emodul: f64, b: f64, h1: f64, h2: f64, k: f64) -> f64 {
    golden_contraction_optimizer(0.0, l, &|x| schnittmoment(q, l, emodul, b, h1, h2, k, x))
}

fn get_max_schnittmoment(q: f64, l: f64, emodul: f64, b: f64, h1: f64, h2: f64, k: f64) -> f64 {
    golden_contraction_optimizer(0.0, l, &|x| -schnittmoment(q, l, emodul, b, h1, h2, k, x))
}

fn get_min_vec(l: f64, b: f64, fuzzy: &Vec<f64>) -> f64 {
    get_min_schnittmoment(fuzzy[0], l, fuzzy[1], b, fuzzy[2], fuzzy[3], fuzzy[4])
}

fn get_max_vec(l: f64, b: f64, fuzzy: &Vec<f64>) -> f64 {
    -get_max_schnittmoment(fuzzy[0], l, fuzzy[1], b, fuzzy[2], fuzzy[3], fuzzy[4])
}

pub fn main() {
    let b = 0.2;
    let l = 10.0;
    let linienlast = FuzzyTrapezoidal::new(14.0, 17.0, 18.5, 22.0);
    let emodul = FuzzyTrapezoidal::new(2.05e8, 2.1e8, 2.12e8, 2.15e8);
    let h1 = FuzzyTriangular::new(0.65, 0.7, 0.75);
    let h2 = FuzzyTriangular::new(0.48, 0.5, 0.52);
    let federsteif = FuzzyTriangular::new(0.8e4, 1.0e4, 1.3e4);

    let func1 = move |fuz: &Vec<f64>| get_min_vec(l, b, fuz);
    let func2 = move |fuz: &Vec<f64>| get_max_vec(l, b, fuz);

    let fu = FuzzyAnalysis {
        fuzzy_vars: vec![
            Box::new(linienlast),
            Box::new(emodul),
            Box::new(h1),
            Box::new(h2),
            Box::new(federsteif),
        ],
        output_functions: vec![Box::new(func1), Box::new(func2)],
    };

    let alpha_levels = vec![0.0, 0.2, 0.4, 0.6, 0.8, 1.0];

    for alpha in alpha_levels {
        let res = fu.alpha_level_optimize(alpha);

        println!(
            "{},{},{},{},{}",
            alpha, res.0[0], res.1[0], res.0[1], res.1[1]
        );
    }
}
