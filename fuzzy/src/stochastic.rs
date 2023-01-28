use std::f64::consts::PI;

use rand::{Rng, RngCore};
use rand_distr::Distribution;
use rand_distr::Normal;

pub trait StochasticVariable<R: Rng + ?Sized> {
    fn get_random_sample(&self, randomness: &mut R) -> f64;
    //fn get_sample_deterministic(&self) -> f64;
}

pub struct NormalDistributedVariable {
    distr: Normal<f64>,
}

impl NormalDistributedVariable {
    pub fn new(mean_value: f64, standard_deviation: f64) -> Self {
        return NormalDistributedVariable {
            distr: Normal::new(mean_value, standard_deviation).unwrap(),
        };
    }
}

impl<R: Rng  + ?Sized> StochasticVariable<R> for NormalDistributedVariable {
    fn get_random_sample(&self, randomness: &mut R) -> f64 {
        return self.distr.sample(randomness);
    }
}

#[derive(Debug)]
pub struct ExperimentalCumulativeDistributionFunction {
    samples: Vec<f64>,
}

type ECDF = ExperimentalCumulativeDistributionFunction;

impl ECDF {
    pub fn new(samples: Vec<f64>) -> Self {
        return ECDF { samples };
    }

    pub fn get_samples(&self) -> &Vec<f64> {
        return &self.samples;
    }

    pub fn get_samples_cumulative(&self) -> Vec<[f64; 2]> {
        let n = &self.samples.len();
        let step = 1.0 / (*n as f64);
        let mut res = Vec::with_capacity(*n);

        for i in 0..*n {
            let current_step = step * (i + 1) as f64;
            res.push([self.samples[i], current_step]);
        }

        return res;
    }

    pub fn quantile(&self, quant: f64) -> f64 {
        let step = 1.0/(self.samples.len() as f64);
        let amount = self.samples.len();
        let approx_index = (quant / step).floor() as usize;
        // fallunterscheidung:
        if approx_index <= 0 {
            return self.samples[0];
        } else if (1..amount).contains(&approx_index) {
            let approx_ind = approx_index as f64;
            return self.samples[approx_index]  + (step)/(self.samples[approx_index+1] - self.samples[approx_index])*(quant -step * approx_ind);
        } else {
            return self.samples[self.samples.len()-1];
        }
    }

    pub fn mean(&self) -> f64 {
        let sum: f64 = self.samples.iter().sum();
        let size = self.samples.len() as f64;
        return sum/size;
    }
}

pub trait StochasticAnalysis<R: Rng  + ?Sized> {
    fn get_distributions(&self) -> Vec<Box<dyn StochasticVariable<R>>>;
    fn output_function(&self, input_vec: &Vec<f64>) -> f64;
    //
    fn stochastics_analysis(
        &self,
        level_reliability: f64,
        confidence_interval: f64,
        threshold_probability: f64,
        randomness: &mut R,
    ) -> ECDF {
        let n_sim = (1.0 / (1.0 - level_reliability)
            * threshold_probability
            * (1.0 - threshold_probability)
            / (confidence_interval.powi(2))) as usize;
        let mut res_vec = Vec::with_capacity(n_sim);
        for _ in 0..n_sim {
            let samples: Vec<f64> = self
                .get_distributions()
                .iter()
                .map(|f| f.get_random_sample(randomness))
                .collect();
            let erg = self.output_function(&samples);
            res_vec.push(erg);
        }

        res_vec.sort_by(|a, b| a.partial_cmp(b).unwrap());

        return ECDF::new(res_vec);
    }
}
