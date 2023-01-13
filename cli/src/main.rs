use fuzzy::{
    fuzzy::*,
    stochastic::{
        ExperimentalCumulativeDistributionFunction, NormalDistributedVariable, StochasticAnalysis,
        StochasticVariable,
    },
};
use plotters::prelude::*;
use rand::{rngs::ThreadRng, thread_rng, Rng};

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

pub fn main2() {
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

pub struct Biegebalken {}

impl<R: Rng> StochasticAnalysis<R> for Biegebalken {
    fn get_distributions(&self) -> Vec<Box<dyn StochasticVariable<R>>> {
        return vec![Box::new(NormalDistributedVariable::new(30.0, 20.0))];
    }
    fn output_function(&self, input_vec: &Vec<f64>) -> f64 {
        return input_vec[0] + 3.0 - input_vec[0].cos();
    }
}

fn plot_ecdf(data: &ExperimentalCumulativeDistributionFunction) {
    let max: f64 = data.get_samples()[data.get_samples().len() - 1];
    let min: f64 = data.get_samples()[0];

    let root = BitMapBackend::new("0.png", (640, 480)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let mut chart = ChartBuilder::on(&root)
        .caption("eCDF", ("sans-serif", 50).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(min..max, -0.1f64..1f64)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            data.get_samples_cumulative().iter().map(|f| (f[0], f[1])),
            &RED,
        ))
        .unwrap()
        .label("eCDF")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()
        .unwrap();

    root.present().unwrap();
}

pub fn main() {
    let mut rng = thread_rng();
    let b = Biegebalken {};
    let ecdf = b.stochastics_analysis(0.95, 1e-3, 0.95, &mut rng);
    println!("{:?}", ecdf.get_samples().len());
    println!("{}", ecdf.quantile(0.1));
    println!("{}", ecdf.quantile(0.2));
    println!("{}", ecdf.quantile(0.3));
    println!("{}", ecdf.quantile(0.4));
    println!("{}", ecdf.quantile(0.5));
    println!("{}", ecdf.quantile(0.6));
    println!("{}", ecdf.quantile(0.7));
    println!("{}", ecdf.quantile(0.8));
    println!("{}", ecdf.quantile(0.95));
    plot_ecdf(&ecdf);
}
