

pub trait StochasticVariable {
    fn get_random_sample(&self) -> f64;
    //fn get_sample_deterministic(&self) -> f64;
}

pub struct NormalDistributedVariable {
    mean_value: f64,
    standard_deviation: f64
}

impl NormalDistributedVariable {
    pub fn new(mean_value: f64, standard_deviation: f64) -> Self {
        return NormalDistributedVariable {
            mean_value,
            standard_deviation
        };
    }
}

impl StochasticVariable for NormalDistributedVariable {
    fn get_random_sample(&self) -> f64 {
        todo!("Select Random Sample not implemented!")
    }
}

pub trait StochasticAnalysis {
    fn get_distributions(&self) -> Vec<Box<dyn StochasticVariable>>;
    fn output_function(&self, input_vec: &Vec<f64>) -> f64;
}