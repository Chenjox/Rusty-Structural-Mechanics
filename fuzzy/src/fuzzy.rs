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
            alpha_level * (self.middle_value - self.lower_value) + self.lower_value,
            (1.0 - alpha_level) * (self.upper_value - self.middle_value) + self.middle_value,
        );
    }
}

pub struct FuzzyTrapezoidal {
    lower_value: f64,
    lower_middle_value: f64,
    upper_middle_value: f64,
    upper_value: f64,
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

pub trait FuzzyVariable {
    fn get_support(&self) -> (f64, f64);

    fn membership_of(&self, value: f64) -> f64;

    fn alpha_level_support(&self, alpha_level: f64) -> (f64, f64);
}

pub fn alpha_level_optimize<F>(fuzz: impl FuzzyVariable, alpha_level: f64, func: F) -> (f64, f64)
where
    F: Fn(f64) -> f64,
{
    // Verfahren des Goldenen Schnitts.
    let suchraum = fuzz.alpha_level_support(alpha_level);
    // Minimum
    let max_iter = 10000;
    let mut iter = 0;
    // Startvariablen
    let mut lower_guess = suchraum.0;
    let mut upper_guess = suchraum.1;
    let mut x1 = lower_guess + (1.0 - SMALL_PHI) * (upper_guess - lower_guess);
    let mut x2 = lower_guess + SMALL_PHI * (upper_guess - lower_guess);
    let mut y1 = func(x1);
    let mut y2 = func(x2);
    // Minimum
    while upper_guess - lower_guess > 0.0001 && iter < max_iter {
        if y1 < y2 {
            upper_guess = x2;
            x2 = x1 + (1.0 - SMALL_PHI) * (upper_guess - x1);
            y2 = func(x2);
        } else if y2 > y1 {
            lower_guess = x1;
            x1 = lower_guess + SMALL_PHI * (x2 - lower_guess);
            y1 = func(x1);
        }
        iter += 1;
    }

    let min = lower_guess;
    // Maximum
    let mut lower_guess = suchraum.0;
    let mut upper_guess = suchraum.1;
    let mut x1 = lower_guess + (1.0 - SMALL_PHI) * (upper_guess - lower_guess);
    let mut x2 = lower_guess + SMALL_PHI * (upper_guess - lower_guess);
    let mut y1 = func(x1);
    let mut y2 = func(x2);
    let mut iter = 0;

    while upper_guess - lower_guess > 0.0001 && iter < max_iter {
        if y1 > y2 {
            upper_guess = x2;
            x2 = x1 + (1.0 - SMALL_PHI) * (upper_guess - x1);
            y2 = func(x2);
        } else if y2 < y1 {
            lower_guess = x1;
            x1 = lower_guess + SMALL_PHI * (x2 - lower_guess);
            y1 = func(x1);
        }
        iter += 1;
    }

    let max = upper_guess;

    (min, max)
}
