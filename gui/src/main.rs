use structmech::stiffness::direct_stiffness;
use structmech::stiffness::system::*;

use gui::visual::Visualizeable;

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
                Crosssection::new(2.1e8, 3.0 / 1000.0, 6.0 / 10000.0),
                [false, false, false, false, false, true],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
            Beam::new(
                Crosssection::new(2.1e8, 3.0 / 1000.0, 6.0 / 10000.0),
                [false, false, false, false, false, false],
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
    /*
    let system = System::new(
        vec![
            Point::new(0.0, 5.0),
            Point::new(0.0, 10.0),
            Point::new(5.0, 5.0),
            Point::new(0.0, 0.0),
        ],
        vec![[0, 1], [0, 2], [0, 3]],
        vec![
            Beam::new(
                Crosssection::new(2.1e8, 5e-3, 3.6e-5),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
            Beam::new(
                Crosssection::new(2.1e8, 5e-3, 3.6e-5),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
            Beam::new(
                Crosssection::new(2.1e8, 5e-3, 3.6e-5),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
        ],
        vec![1, 2, 3],
        vec![
            Support::new(0.0, [false, true, false], [0.0, 0.0, 0.0]),
            Support::new(0.0, [true, false, true], [0.0, 0.0, 0.0]),
            Support::new(0.0, [false, false, false], [0.0, 0.0, 0.0]),
        ],
    );

    let system_loading = SystemLoading::new(
        vec![1, 2],
        vec![
            StaticLoad::new(0.0, 300.0, 0.0),
            StaticLoad::new(-100.0, 0.0, 0.0),
        ],
        vec![1],
        vec![StaticLinearLineload::new_linear_load(5.0, 3.0)],
    );
    */

    system.global_stiffness_matrix(&system_loading);
    println!("{}", system.visualize());
}
