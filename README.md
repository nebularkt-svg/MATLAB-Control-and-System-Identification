# MATLAB Control and System Identification

Three simulation studies from **ELECTENG 704 Assignment 1**. The scripts model nonlinear magnetic levitation, analyse sliding-mode chattering and robustness, and identify a simulated plant from input–output data.

[Run the scripts](#run-the-simulations) · [View the reported results and figures](results/README.md)

The reported benchmark includes three useful comparisons: gap tracking under plant mismatch, tracking error versus chattering for different switching laws, and ARX validation fit versus model order. In the ARX experiment, the `[6, 6, 1]` model gave the best reported validation fit at **40.24%**.

| Assignment part | MATLAB script | Simulation test |
| --- | --- | --- |
| Q1(f) | [Maglev simulation](Q1f_Maglev_Simulation.m) | Tests integral-augmented feedback linearisation on the nonlinear plant with a 20% mass mismatch, damping, and a voltage limit. |
| Q2(c–d) | [Sliding-mode simulation](Q2_SlidingMode_Simulation.m) | Compares discontinuous switching with two boundary-layer widths and tests the effect of plant parameter mismatch. |
| Q3 | [ARX identification](Q3_ARX_Identification.m) | Excites the plant with a pseudorandom binary sequence (PRBS), fits four ARX models, and compares validation fit and frequency response. |

[Assignment results](results/README.md) contains the reported values and six figures extracted from the assignment PDF.

## Run the simulations

Open MATLAB in the repository directory and run the scripts below individually. Each script creates its figures and prints the numerical results in the Command Window.

```matlab
Q1f_Maglev_Simulation
Q2_SlidingMode_Simulation
Q3_ARX_Identification
```

The ARX script requires System Identification Toolbox and Control System Toolbox. The archived figures and reported values come from the assignment submission.
