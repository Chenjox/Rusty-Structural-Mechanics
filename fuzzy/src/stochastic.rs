use std::f64::consts::PI;

use rand::{Rng, RngCore};
use rand_distr::Normal;
use rand_distr::Distribution;


fn inverse_erf(z: f64) -> f64 {
    return PI.sqrt() * 0.5 * ( z 
        + PI * 1.0/12.0 * z.powi(3) 
        + 7.0 / 480.0 * PI * PI * z.powi(5)
        + 127.0 / 40320.0 * PI * PI * PI * z.powi(7)
        + 4369.0 / 5806080.0 * PI * PI * PI * PI * z.powi(9) );
}

pub trait StochasticVariable<R: Rng> {
    fn get_random_sample(&self, randomness: &mut R) -> f64;
    //fn get_sample_deterministic(&self) -> f64;
}

pub struct NormalDistributedVariable {
    distr: Normal<f64>
}

impl NormalDistributedVariable {
    pub fn new(mean_value: f64, standard_deviation: f64) -> Self {
        return NormalDistributedVariable {
            distr: Normal::new(mean_value, standard_deviation).unwrap()
        };
    }
}

impl<R: Rng> StochasticVariable<R> for NormalDistributedVariable {
    fn get_random_sample(&self, randomness: &mut R) -> f64 {
        return self.distr.sample(randomness);
    }
}

pub trait StochasticAnalysis<R: Rng> {
    fn get_distributions(&self) -> Vec<Box<dyn StochasticVariable<R>>>;
    fn output_function(&self, input_vec: &Vec<f64>) -> f64;
}