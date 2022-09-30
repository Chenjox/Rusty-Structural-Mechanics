use structmech::stiffness::direct_stiffness;
use structmech::stiffness::system::*;

fn main() {
    let system = System::new(
        vec![
            Point::new(0.0, 0.0),
            Point::new(0.0, 4.0),
            Point::new(4.0, 4.0),
        ],
        vec![[0, 1], [1, 2]],
        vec![
            Beam::new(
                Crosssection::new(2e8, 4e-4, 1e-4),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
            Beam::new(
                Crosssection::new(2e8, 4e-4, 1e-4),
                [false, false, true, false, false, true],
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

    system.global_stiffness_matrix();
}
