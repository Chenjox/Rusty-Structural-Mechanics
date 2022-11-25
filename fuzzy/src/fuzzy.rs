use crate::optimization;

const SMALL_PHI: f64 = 0.618033988749894848204586834365638118_f64;

pub struct FuzzyTriangular {
    lower_value: f64,
    middle_value: f64,
    upper_value: f64,
}

impl FuzzyVariable for FuzzyTriangular {
    fn get_support(&self) -> (f64, f64) {
        return (self.lower_value, self.upper_value);
    }

    fn membership_of(&self, value: f64) -> f64 {
        let lower = self.lower_value;
        let middle = self.middle_value;
        let upper = self.upper_value;
        if value == middle {
            1.0
        } else if value >= lower && value < middle {
            (value - lower) / (middle - lower)
        } else if value >= middle && value < lower {
            1.0 - (value - middle) / (upper - middle)
        } else {
            0.0
        }
    }

    fn alpha_level_support(&self, alpha_level: f64) -> (f64, f64) {
        return (
            self.lower_value + alpha_level * (self.middle_value - self.lower_value),
            self.upper_value - alpha_level * (self.upper_value - self.middle_value),
        );
    }
}

impl FuzzyTriangular {
    pub fn new(lower: f64, middle: f64, upper: f64) -> Self {
        return FuzzyTriangular {
            lower_value: lower,
            middle_value: middle,
            upper_value: upper,
        };
    }
}

pub struct FuzzyTrapezoidal {
    lower_value: f64,
    lower_middle_value: f64,
    upper_middle_value: f64,
    upper_value: f64,
}

impl FuzzyTrapezoidal {
    pub fn new(lower: f64, middle_lower: f64, middle_upper: f64, upper: f64) -> Self {
        return FuzzyTrapezoidal {
            lower_value: lower,
            lower_middle_value: middle_lower,
            upper_middle_value: middle_upper,
            upper_value: upper,
        };
    }
}

impl FuzzyVariable for FuzzyTrapezoidal {
    fn get_support(&self) -> (f64, f64) {
        return (self.lower_value, self.upper_value);
    }

    fn membership_of(&self, value: f64) -> f64 {
        let lower = self.lower_value;
        let lmiddle = self.lower_middle_value;
        let umiddle = self.upper_middle_value;
        let upper = self.upper_value;
        if value >= lmiddle && value <= umiddle {
            1.0
        } else if value >= lower && value < lmiddle {
            (value - lower) / (lmiddle - lower)
        } else if value >= umiddle && value < lower {
            1.0 - (value - umiddle) / (upper - umiddle)
        } else {
            0.0
        }
    }

    fn alpha_level_support(&self, alpha_level: f64) -> (f64, f64) {
        return (
            alpha_level * (self.lower_middle_value - self.lower_value) + self.lower_value,
            (1.0 - alpha_level) * (self.upper_value - self.upper_middle_value)
                + self.upper_middle_value,
        );
    }
}

pub struct FuzzyAnalysis {
    pub fuzzy_vars: Vec<Box<dyn FuzzyVariable>>,
    pub output_functions: Vec<Box<dyn Fn(&Vec<f64>) -> f64>>,
}

impl FuzzyAnalysis {
    pub fn alpha_level_optimize(&self, alpha_level: f64) -> (Vec<f64>, Vec<f64>) {
        let mut lower_bound = vec![0.0; self.fuzzy_vars.len()];
        let mut upper_bound = vec![0.0; self.fuzzy_vars.len()];

        // Fuzzy Größen
        for i in 0..self.fuzzy_vars.len() {
            let (low, high) = self.fuzzy_vars[i].alpha_level_support(alpha_level);
            lower_bound[i] = low;
            upper_bound[i] = high;
        }
        // Startpunkt in der Mitte
        let mut initial = vec![0.0; self.fuzzy_vars.len()];
        for i in 0..self.fuzzy_vars.len() {
            initial[i] = 0.5 * lower_bound[i] + 0.5 * upper_bound[i];
        }

        let mut minres = vec![0.0; self.output_functions.len()];
        let mut maxres = vec![0.0; self.output_functions.len()];

        // Jede Zielfunktion muss maximiert//minimiert werden werden
        for func in 0..self.output_functions.len() {
            // Minimum
            let r_min = optimization::simplex_optimization(
                &lower_bound,
                &upper_bound,
                &initial,
                &self.output_functions[func],
            );

            // Maximum
            let f = |vl: &Vec<f64>| (-&self.output_functions[func](vl));
            let r_max = optimization::simplex_optimization(&lower_bound, &upper_bound, &initial, f);
            minres[func] = self.output_functions[func](&r_min);
            maxres[func] = self.output_functions[func](&r_max);
        }

        return (minres, maxres);
    }
}

pub trait FuzzyVariable {
    fn get_support(&self) -> (f64, f64);

    fn membership_of(&self, value: f64) -> f64;

    fn alpha_level_support(&self, alpha_level: f64) -> (f64, f64);
}

pub fn alpha_level_optimize<F>(fuzz: &impl FuzzyVariable, alpha_level: f64, func: F) -> (f64, f64)
where
    F: Fn(f64) -> f64,
{
    // Verfahren des Goldenen Schnitts.
    let suchraum = fuzz.alpha_level_support(alpha_level);
    // Minimum
    let min = golden_contraction_optimizer(suchraum.0, suchraum.1, &func);
    // Maximum
    let max = -golden_contraction_optimizer(suchraum.0, suchraum.1, &|x| -func(x));

    (min, max)
}

pub fn golden_contraction_optimizer<F>(mut lower_guess: f64, mut upper_guess: f64, func: &F) -> f64
where
    F: Fn(f64) -> f64,
{
    let eps = 0.00000001;
    // Minimum
    let max_iter = 100000;
    let mut x1 = lower_guess + (1.0 - SMALL_PHI) * (upper_guess - lower_guess);
    let mut x2 = lower_guess + SMALL_PHI * (upper_guess - lower_guess);
    let mut y1 = func(x1);
    let mut y2 = func(x2);
    let mut iter = 1;

    loop {
        if y1 < y2 {
            upper_guess = x2;
            x2 = x1;
            y2 = y1;
            x1 = lower_guess + SMALL_PHI * (x2 - lower_guess);
            y1 = func(x1);
        } else {
            // if y2 > y1
            lower_guess = x1;
            x1 = x2;
            y1 = y2;
            x2 = x1 + (1.0 - SMALL_PHI) * (upper_guess - x1);
            y2 = func(x2);
        }
        iter += 1;
        if !((upper_guess - lower_guess).abs() > eps && iter < max_iter) {
            break;
        }
    }

    func(upper_guess)
}
