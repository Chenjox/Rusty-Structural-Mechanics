use structmech::stiffness::direct_stiffness;
use structmech::stiffness::system::*;

fn main() {
    let system = System::new(
        vec![
            Point::new(0.0, 0.0),
            Point::new(3.0, 4.0),
            Point::new(9.0, 4.0),
        ],
        vec![[0, 1], [1, 2]],
        vec![
            Beam::new(
                Crosssection::new(2.1e8, 3e-3, 6e-4),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
            Beam::new(
                Crosssection::new(2.1e8, 3e-3, 6e-4),
                [false, false, true, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
        ],
        vec![0, 2],
        vec![
            Support::new(0.0, [false, false, false], [0.0, 0.0, 0.0]),
            Support::new(0.0, [false, false, false], [0.0, 0.0, 0.0]),
        ],
    );

    let system_loading = SystemLoading::new(
        vec![1],
        vec![StaticLoad::new(0.0, 100.0, 0.0)],
        vec![1],
        vec![StaticLinearLineload::new_constant_load(20.0)],
    );

    system.global_stiffness_matrix(&system_loading);
}
