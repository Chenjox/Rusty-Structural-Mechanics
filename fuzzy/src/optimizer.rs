pub fn optimize_downhill_simplex<F>(
    lower_bound: &[f64],
    upper_bound: &[f64],
    start: &[f64],
    func: &F,
) -> Vec<f64>
where
    F: Fn(&[f64]) -> f64,
{
    fn check_in_bounds(n: usize, low: &[f64], up: &[f64], b: &[f64]) -> bool {
        for i in 0..n {
            if b[i] < low[i] || b[i] > up[i] {
                return false;
            }
        }
        return true;
    }
    //
    let n = lower_bound.len();
    let mut points = Vec::with_capacity(n + 1);
    // Anfangspunkte
    {
        // Werden mittels lerp erstellt.
        for i in 0..n {
            // nehme einen Schritt im Bereich
            let interval = (start[i] + lower_bound[i]) * 0.5;
            // erstelle einen neuen punkt
            let mut new_point = vec![0.0; n];
            for k in 0..n {
                // der in der Koordinate k vom startpunkt abweicht
                if k == i {
                    new_point[k] = start[k] + interval
                } else {
                    // und sonst gleich ist.
                    new_point[k] = start[k]
                }
            }
            // und füge Ihn zur der point liste hinzu
            points.push(new_point);
        }
    }
    // letztlich der startpunkt
    points.push(Vec::from(start));
    // Nun berechne alle Funktionswerte
    let mut point_results = vec![0.0; n + 1];
    for i in 0..n + 1 {
        point_results[i] = func(&points[i]);
    }
    // sortiere die Punkte nach den Funktionswerten
    let mut point_indeces = [0, n - 1, n]; // 0 ist der beste, 1 der zweitschlechteste, 2 der schlechteste
    let mut iter = 0;
    let max_iter = 100;
    'out: loop {
        println!("{:?}", points);
        println!("{:?}", point_results);
        iter += 1;
        {
            // Sorting sweep
            let mut f_best = point_results[0];
            let mut f_second_worst = point_results[n - 1];
            let mut f_worst = point_results[n];
            for i in 0..n + 1 {
                // finden von minimum und maximum
                if f_best < point_results[i] {
                    // ist der angenommene beste wert kleiner als der momentane?
                    f_best = point_results[i]; // wenn ja dann ist ja gut.
                    point_indeces[0] = i;
                }
                if f_worst > point_results[i] {
                    // ist der angenommene schlechteste wert größer als der momentane?
                    f_worst = point_results[i]; // wenn ja, neuer vergleichswert, sonst nicht
                    point_indeces[2] = i;
                }
            }
            // findes des zweitschlechtesten werts
            for i in 0..n + 1 {
                // finden von minimum und maximum
                if i != point_indeces[2] && f_second_worst > point_results[i] {
                    // ist der angenommene schlechteste wert größer als der momentane?
                    f_second_worst = point_results[i]; // wenn ja, neuer vergleichswert, sonst nicht
                    point_indeces[1] = i;
                }
            }
        }
        // Überprüfen des Abbruchkriteriums:
        {
            let mut abbruch = true;
            for i in 1..n + 1 {
                // alle punkte, mitaußnahme des ersten
                for k in 0..n {
                    // alle koordinaten
                    if (points[0][k] - points[i][k]).abs() > 0.000001 {
                        abbruch = false;
                    }
                }
            }
            if abbruch || iter > max_iter {
                break 'out;
            }
        }
        // alle werte gefunden:

        // bilden des mittelpunktes
        let mut mittelpunkt = vec![0.0; n];
        for i in 0..n {
            let mut coord = 0.0;
            for k in 0..n + 1 {
                if k != point_indeces[2] {
                    coord += points[k][i];
                }
            }
            let coord = coord / (n as f64);
            mittelpunkt[i] = coord;
        }
        // reflexion
        {
            let mut r = vec![0.0; n];
            for i in 0..n {
                r[i] = mittelpunkt[i] + 1.0 * (mittelpunkt[i] - points[point_indeces[2]][i]);
            }
            let fr = func(&r);

            if check_in_bounds(n, &lower_bound, &upper_bound, &r)
                && fr > point_results[point_indeces[0]]
            {
                // ist fr besser als das momentan beste
                let mut e = vec![0.0; n];
                for i in 0..n {
                    // expansion
                    e[i] = r[i] + 1.0 * (r[i] - mittelpunkt[i]);
                }
                let fe = func(&e);
                if check_in_bounds(n, &lower_bound, &upper_bound, &e) && fe > fr {
                    // ersetze schlechtesten Punkt mit dem besten der beiden punkte
                    points[point_indeces[2]] = e;
                    point_results[point_indeces[2]] = fe;
                } else {
                    points[point_indeces[2]] = r;
                    point_results[point_indeces[2]] = fr;
                }
                // gehe zur nächsten iteration!
                continue 'out;
            } else if check_in_bounds(n, &lower_bound, &upper_bound, &r)
                && fr > point_results[point_indeces[1]]
            {
                points[point_indeces[2]] = r;
                point_results[point_indeces[2]] = fr;
                // gehe zur nächsten iteration!
                continue 'out;
            } else {
                // kontraktion!
                let mut c = vec![0.0; n];
                for i in 0..n {
                    if check_in_bounds(n, &lower_bound, &upper_bound, &r)
                        && fr > point_results[point_indeces[2]]
                    {
                        // r ist besser als das schlechteste
                        c[i] = r[i] + 0.5 * (mittelpunkt[i] - r[i]);
                    } else {
                        // das schlechteste ist besser als r
                        c[i] = points[point_indeces[2]][i]
                            + 0.5 * (mittelpunkt[i] - points[point_indeces[2]][i]);
                    }
                }
                let fc = func(&c);
                if check_in_bounds(n, &lower_bound, &upper_bound, &c)
                    && fc > point_results[point_indeces[2]]
                {
                    // fc ist besser als das schlechteste
                    points[point_indeces[2]] = c;
                    point_results[point_indeces[2]] = fc;
                    // zur nächsten Iteration
                    continue 'out;
                } else {
                    // suchraum verkleinern
                    for i in 1..n + 1 {
                        if i != point_indeces[0] {
                            // wenn es nicht das beste ist
                            let mut new_point = vec![0.0; n];
                            for k in 0..n {
                                new_point[k] = points[i][k]
                                    + 0.5 * (points[point_indeces[0]][k] - points[i][k]);
                            }
                            point_results[i] = func(&new_point);
                            points[i] = new_point;
                        }
                    }
                    // gehe zur nächsten iteration
                    continue 'out;
                }
            }
        }
    }
    return points[point_indeces[0]].clone();
} // ternary heap for better caching...
