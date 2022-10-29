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
