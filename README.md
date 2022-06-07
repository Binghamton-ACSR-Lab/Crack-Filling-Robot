# Crack-Filling-Robot

> **Complete and Near-Optimal Coverage Planning and  Control in Robotic Crack Filling**
>
> Video : [We made a Crack Filling Robot！ - YouTube](https://www.youtube.com/watch?v=VYJV3xwJ_5Y)

> ![](Docs/1.Images/WallE1.JPG)

<br/>

## Abstract
<div style="text-align: justify">
We present a simultaneous sensor-based inspection and footprint coverage (RIFC) planning and robotic control design with applications to efficiently detect and fill unknown cracks in civil infrastructure. Surface cracks exist in many civil infrastructures, such as road and bridge deck surfaces, parking plots, etc., and the RIFC design enables a robotic crack detection and repair system. The proposed online sensor-based complete coverage planning minimizes the total traveling distance of the robot, guarantees the complete sensor coverage of the entire workspace, and achieves near-optimal footprint coverage and repairing of all the cracks. The new SIFC algorithm constructs the unknown crack maps and Reed graphs online. The total distance traveled by the robot is minimized and the resulting near-optimal path is calculated in polynomial time. The onboard motion control is coordinated with the planned robot trajectory to efficiently fill all cracks underneath the robot footprint. Extensive experimental results are presented to demonstrate the system design and performance evaluation. The SIFC design provide a high-efficiency online planning algorithmic solution to infrastructure maintenance and other robotic applications.
</div>

## Algorithm Design Overview

To solve the above SIFC problem, we present a set of algorithmic developments in this paper.
> ![](Docs/2.Drawings/algorithm_overview.drawio.svg)

* ### Sensor-based Complete Coverage (SCC)
    We discuss the optimal coverage planning with known cracks in the previous section, and the crack coverage planning algorithm does not consider the sensor coverage. In order to solve the SIFT problem, we first consider the coverage planning with known target (crack) information. Then, we generalize the algorithm to the case with unknown target information.
* ### online Sensor-based Complete Coverage (oSCC)
    The onlineSCC algorithm is a practical extension of SCC where the robot stores and incrementally constructs the crack graph online. It scans for new cracks in W and updates the crack graph while simultaneously filling it.

## Flow Chart
| | |
|:----:|:----:|
| SCC Flow Cahrt| SCC Simulation Result |
| <img width="100%" src="Docs/2.Drawings/SCC_flow.svg"> | <img width="600" src="Results/SCC/GIF/myCrack8_100_1_SCC.gif"> |
| oSCC Flow Cahrt| oSCC Simulation Result |
| <img width="100%" src="Docs/2.Drawings/oSCC_flow.svg"> | <img width="600" src="Results/oSCC/GIF/myCrack8_100_1_oSCC.gif"> |

<br/>

## Design Overview

### Hardware:
* Eg. Jetson Nano
* Eg. Lidar
  
### Software:
* MATLAB
* LabVIEW

> ![](Docs/1.Images/WallE_CAD.png)
> ![](Docs/1.Images/WallE_CAD_front.png)
> ![](Docs/1.Images/WallE_CAD_top.png)
> ![](Docs/1.Images/WallE_CAD_top_open.png)


## Experiment Performance Comparison on Four Crack Maps under Five Planning Algorithms：

|             |         Fililng Time (s)         ||||          Robot Travel Time (s)          ||||           Robot Path Length (m)            ||||         Nozzle Path (m)         ||||           Sensor Coverage (%)            ||||          Filling Accuracy (%)          ||||   |
| :---------- | :--------------: | :-: | :--: | :-: | :-------------------: | :--: | :--: | :--: | :-------------------: | :---: | :---: | :---: | :-------------: | :-: | :--: | :-: | :-----------------: | :---: | :---: | :---: | :------------------: | :--: | :--: | :--: | - |
|             |       U100       | U80 | G100 | G20 |         U100          | U80  | G100 | G20  |         U100          |  U80  | G100  |  G20  |      U100       | U80 | G100 | G20 |        U100         |  U80  | G100  |  G20  |         U100         | U80  | G100 | G20  |   |
| `onlineSCC` |       731        | 401 | 356  | 346 |         1398          | 937  | 952  | 889  |          51           |  42   |  52   |  46   |       26        | 15  |  15  | 14  |         131         |  109  |  133  |  119  |         98.9         | 98.9 | 98.1 | 98.1 |   \
|             |                  |     |      |     |                       |      |      |      |         (50)          | (40)  | (45)  | (44)  |                 |     |      |     |        (129)        | (103) | (117) | (114) |                      |      |      |      |   |
| `SCC`       |       654        | 430 | 366  | 345 |         1328          | 975  | 919  | 889  |          51           |  49   |  49   |  52   |       23        | 16  |  14  | 12  |         132         |  126  |  127  |  134  |         99.1         | 99.8 | 98.4 | 98.4 |   \
|             |                  |     |      |     |                       |      |      |      |         (48)          | (43)  | (46)  | (45)  |                 |     |      |     |        (123)        | (111) | (118) | (117) |                      |      |      |      |   |
| `GCC`       |       559        | 364 | 273  | 276 |          914          | 577  | 455  | 455  |          30           |  19   |  15   |  13   |       30        | 20  |  14  | 12  |         58          |  34   |  24   |  20   |         99.1         | 99.8 | 98.4 | 98.4 |   \
|             |                  |     |      |     |                       |      |      |      |         (29)          | (17)  | (13)  | (12)  |                 |     |      |     |        (60)         | (34)  | (24)  | (20)  |                      |      |      |      |   |
| `Greeedy`   |       746        | 446 | 414  | 372 |         1714          | 1228 | 1104 | 1060 |          73           |  63   |  58   |  57   |       29        | 19  |  17  | 16  |         187         |  163  |  150  |  147  |         99.6         | 99.5 | 99.1 | 99.1 |   \
|             |                  |     |      |     |                       |      |      |      |         (65)          | (53)  | (53)  | (48)  |                 |     |      |     |        (116)        | (137) | (137) | (123) |                      |      |      |      |   |
| `ZigZag`    |       752        | 480 | 448  | 415 |         2611          | 2284 | 2149 | 2166 |          196          |  195  |  195  |  195  |       29        | 19  |  16  | 18  |         504         |  502  |  501  |  501  |         99.1         | 99.8 | 98.4 | 98.4 |   \
|             |                  |     |      |     |                       |      |      |      |         (201)         | (201) | (201) | (201) |                 |     |      |     |        (517)        | (517) | (517) | (517) |                      |      |      |      |   |


<br/>
