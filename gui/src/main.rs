use std::f64::consts;
use std::fs::File;
use std::io::prelude::*;
use std::path::Path;
use structmech::stiffness;
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

    //println!("{}", system.visualize());

    let sol = system.matrix_stiffness_method_first_order(&system_loading);
    for i in 0..sol.get_results().len() {
        let v = sol.get_results()[i].get_rsks();
        println!(
            "S_i = {}, Q_i = {}, M_i = {}, S_j = {}, Q_j = {}, M_j = {}",
            v[0], v[1], v[2], v[3], v[4], v[5]
        );
        let v = sol.get_results()[i].get_rvs();
        println!(
            "u_1 = {}, v_1 = {}, phi_1 = {}, u_2 = {}, v_2 = {}, phi_2 = {}",
            v[0], v[1], v[2], v[3], v[4], v[5]
        );
        let r = &sol.get_results()[i];
        let mut resVec = Vec::new();
        for point in 0..11 {
            let inkr = 0.1 * point as f64;
            let v = r.get_internals_at(r.get_beam_lenght() * inkr);
            resVec.push([
                r.get_beam_lenght() * inkr,
                v[0],
                v[1],
                v[2],
                v[3],
                v[4],
                v[5],
            ]);
        }
        write_string_to_file_6(&format!("out{}.csv", i), resVec);
    }

    //system_second_order_stability();
}

fn system_second_order_stability() {
    let system = System::new(
        vec![
            Point::new(0.0, 4.0),
            Point::new(0.0, 7.0),
            Point::new(4.0, 4.0),
            Point::new(0.0, 0.0),
        ],
        vec![[0, 1], [0, 2], [0, 3]],
        vec![
            Beam::new(
                Crosssection::new(2.1e8, 1e-3, 3e-4),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
            Beam::new(
                Crosssection::new(2.1e8, 1e-3, 3e-4),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
            Beam::new(
                Crosssection::new(2.1e8, 1e-3, 1e-4),
                [false, false, false, false, false, false],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                0.0,
                0.0,
            ),
        ],
        vec![0, 1, 2, 3],
        vec![
            Support::new(0.0, [true, true, true], [0.0, 4e4, 0.0]),
            Support::new(0.0, [false, true, false], [0.0, 0.0, 0.0]),
            Support::new(0.0, [true, false, false], [0.0, 0.0, 0.0]),
            Support::new(0.0, [false, false, true], [0.0, 0.0, 0.0]),
        ],
    );

    let mut detvec = Vec::new();

    let max_iter = 1000;
    let mut second_last = 0.0;
    let mut last = 0.0;
    for i in 0..1000 {
        let incr = i as f64 / 10.0;
        let system_loading = SystemLoading::new(
            vec![1, 2],
            vec![
                StaticLoad::new(0.0, 2.0 * 300.0 * incr, 0.0),
                StaticLoad::new(-300.0 * incr, 0.0, 0.0),
            ],
            vec![1],
            vec![StaticLinearLineload::new_linear_load(50.0, 50.0)],
        );

        let mat = system.matrix_stiffness_method_second_order_matrix(&system_loading);
        let d = mat.determinant();

        //println!("{}, {}", 2.0 * 300.0 * incr, d);
        detvec.push([incr, d]);
        second_last = last;
        last = incr;
        if d < 0.0 {
            break;
        }
    }

    let mut unten = second_last;
    let mut oben = last;

    for i in 0..1000 {
        let half = unten + oben / 2.0;

        let system_loading = SystemLoading::new(
            vec![1, 2],
            vec![
                StaticLoad::new(0.0, 2.0 * 300.0 * half, 0.0),
                StaticLoad::new(-300.0 * half, 0.0, 0.0),
            ],
            vec![1],
            vec![StaticLinearLineload::new_linear_load(50.0, 50.0)],
        );

        let mat = system.matrix_stiffness_method_second_order_matrix(&system_loading);
        let d = mat.determinant();

        if d < 0.0 {
            oben = half;
        } else {
            unten = half;
        }
        if i % 100 == 0 {
            println!("{}", half);
        }
    }

    write_string_to_file("determinant.txt", detvec);
}

fn system_second_order() {
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

    let sol = system.matrix_stiffness_method_second_order(&system_loading);
    for i in 0..sol.get_results().len() {
        let v = sol.get_results()[i].get_rsks();
        println!("{}, {}, {}, {}, {}, {}", v[0], v[1], v[2], v[3], v[4], v[5]);
    }
    let sol = system.matrix_stiffness_method_first_order(&system_loading);
    for i in 0..sol.get_results().len() {
        let v = sol.get_results()[i].get_rsks();
        println!("{}, {}, {}, {}, {}, {}", v[0], v[1], v[2], v[3], v[4], v[5]);
    }
}

fn write_string_to_file_6(nam: &str, l: Vec<[f64; 7]>) {
    let r = l
        .iter()
        .map(|f| {
            format!(
                "{},{},{},{},{},{},{}",
                f[0], f[1], f[2], f[3], f[4], f[5], f[6]
            )
        })
        .collect::<Vec<String>>()
        .join("\n");
    let path = Path::new(&nam);
    let mut file = match File::create(&path) {
        Err(why) => {
            //error!("Couldn't create {}: {}", nam, why);
            return;
        }
        Ok(file) => file,
    };
    match file.write_all(r.as_bytes()) {
        Err(why) => {} //error!("couldn't write to {}: {}", nam, why),
        Ok(_) => {}    //info!("successfully wrote to {}", nam),
    }
}

fn write_string_to_file(nam: &str, l: Vec<[f64; 2]>) {
    let r = l
        .iter()
        .map(|f| format!("{},{}", f[0], f[1]))
        .collect::<Vec<String>>()
        .join("\n");
    let path = Path::new(&nam);
    let mut file = match File::create(&path) {
        Err(why) => {
            //error!("Couldn't create {}: {}", nam, why);
            return;
        }
        Ok(file) => file,
    };
    match file.write_all(r.as_bytes()) {
        Err(why) => {} //error!("couldn't write to {}: {}", nam, why),
        Ok(_) => {}    //info!("successfully wrote to {}", nam),
    }
}
