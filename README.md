# PerMedCoE Observatory Unit Tests

See also https://github.com/Chaste/Chaste/issues/166.

## General Comments

* All of the unit tests, as currently specified, are unnecessarily ambiguous. One way to help minimise this ambiguity would be to ask for the Physicell code to be uploaded to the github repo, since Physicell results are included in more results figures than any other tool; of course, ideally, all tools would provide their code for each unit test. Can Arnau ask the Physicell team to provide their code, in the first instance?
 
## Cell Cycle - Fixed Duration

* Ideally, we should record precisely how the `GrowthModifier` class (in particular, the numerical values used to relate cell volume to age) was constructed based on the [experimental dataset](https://github.com/PerMedCoE/observatory_benchmark/blob/main/multiscale_benchmark/2022_09_hackathon/experimental_data/unit_test_cellcycle/Flow%20Cytometry%20Cell%20Cycle%20volume%20dynamics.txt).
* The unit test specification does not unambiguously describe how to specify the cell volume as a function of time, based on the [experimental dataset](https://github.com/PerMedCoE/observatory_benchmark/blob/main/multiscale_benchmark/2022_09_hackathon/experimental_data/unit_test_cellcycle/Flow%20Cytometry%20Cell%20Cycle%20volume%20dynamics.txt) provided. To improve comparison across tools, more detail is needed; ideally, Arnau should either specify cell volume as a function of time, or specify how the data should be interpolated over time (e.g. piecewise linearly).
* Figure 2.1.2.b on page 14 of the [benchmarking report](https://drive.google.com/file/d/1bgpD29n1Wr-scJkfA8KehB2m3UyFOBeB/view?usp=drive_link) does not reflect the Chaste results recorded in the [github repo](https://github.com/PerMedCoE/observatory_benchmark/blob/main/multiscale_benchmark/2022_09_hackathon/Chaste/unit_test_cellcycle/results/cellcycle_fixed.png). Can Arnau update this?

## Cell Cycle - Stochastic Duration

* Similar issues to those raised above.
* The unit test specification does not unambiguously define the stochastic model of cell cycle progression. To improve comparison across tools, more detail is needed; ideally, Arnau should specify whether the cell cycle phase durations are independent random variables.
* The results seem to be based on a single stochastic simulation performed with each tool. To improve comparison across tools, summary statistics should instead be plotted, e.g. mean +/- standard deviation, based on say 10 simulations.

## Diffusion - Single Cell Sink

* We are working towards an implementation of this unit test in Chaste.
* The unit test specification does not unambiguously define the initial condition. Should c(x, 0) be 10 everywhere in the domain, 0 everywhere in the domain, or 10 everywhere in the domain except at the very centre? What is the spatial extent of the sink term? What discretization is required in the case of FEM? These aren't clear as currently described.

## Diffusion - 1k Cells as Sinks

* We are working towards an implementation of this unit test in Chaste.
* Similar issues to those raised above.
  
## Mechanics - Movement of a Cell with Friction

* Now implemented in Chaste.
* For this unit test in particular, it would be good to see the Physicell code so we can work out precisely how they interpreted the unit test specification, so that we can do something as close as possible.
* Figure 2.1.2.e on page 17 of the [benchmarking report](https://drive.google.com/file/d/1bgpD29n1Wr-scJkfA8KehB2m3UyFOBeB/view?usp=drive_link) seems to be a plot of velocity against time, not distance moved against time as claimed. Can Arnau check this?

![results](results/mechanics-single-cell-with-friction/plot.png)
  
## Mechanics - Two Cells Pushing Each Other

* We are working towards an implementation of this unit test in Chaste.

## One Cell with Chemotaxis

## Use Case - 2D Monolayer without Diffusion

## Use Case - Spheroid without Diffusion

## Use Case - Spheroid with Diffusion
