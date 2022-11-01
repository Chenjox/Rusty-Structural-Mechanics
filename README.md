# Rusty Structural Mechanics

## English

This repository contains multiple solvers for problems arising in structural mechanics.

Currently it contains a solver utilizing the matrix stiffness method for `1st order theory` as well as `2nd order theory`.

### Usage

Every Solver needs a `System` and a `Loading`.

```rust
let system = System::new(
    vec![ // Give your points of the system.
        Point::new(0.0, 0.0),
        Point::new(3.0, 4.0),
        Point::new(9.0, 4.0),
    ],
    vec![[0, 1], [1, 2]],
    vec![
        Beam::new( // Now the Beams and their DOFs
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
    vec![0, 2], // Last the supports
    vec![
        Support::new(0.0, [false, false, false], [0.0, 0.0, 0.0]),
        Support::new(0.0, [false, false, false], [0.0, 0.0, 0.0]),
    ],
);

let system_loading = SystemLoading::new( // Next declare a Loading
    vec![1], // This is a point load on point 1
    vec![StaticLoad::new(0.0, 100.0, 0.0)],
    vec![1], // this is a linear load on beam 1
    vec![StaticLinearLineload::new_constant_load(20.0)],
);
```

The solver is a implementations on the system:
```rust
let sol = system.matrix_stiffness_method_first_order(&system_loading);
```

It returns a `BeamResultSet` which contains the boundary internal forces of every beam, stored in a `BeamResult`.
Currently internal forces inside the beam can be calculated with the `BeamResult#get_internals_at(x)` where `x` is the position of the coordinate in the local coordinate system.

```rust
let r = &sol.get_results()[0]; // Select the first beam and borrow it.
let internal_mid = r.get_internals_at(r.get_beam_length() * 0.5); // Calculate the internal forces in the middle.
```

## German

Dieses Repository enthält mehrere Löser von Problemen der Strukturmechanik.

Zum jetztigen Zeitpunkt, wird die vollständige Deformationsmethode benutzt um Schnittkräfte nach Theorie I. Ordnung und Theorie II. Ordnung zu berechnen.
