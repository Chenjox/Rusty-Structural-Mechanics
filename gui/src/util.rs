use std::fs::File;
use std::io::prelude::*;
use std::path::Path;

pub fn write_string_to_file_6(nam: &str, l: Vec<[f64; 7]>) {
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

pub fn write_file(name: &str, s: &str) {
    let path = Path::new(&name);
    let mut file = match File::create(&path) {
        Err(why) => {
            //error!("Couldn't create {}: {}", nam, why);
            return;
        }
        Ok(file) => file,
    };
    match file.write_all(s.as_bytes()) {
        Err(why) => {} //error!("couldn't write to {}: {}", nam, why),
        Ok(_) => {}    //info!("successfully wrote to {}", nam),
    }
}

pub fn write_string_to_file(nam: &str, l: Vec<[f64; 2]>) {
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
