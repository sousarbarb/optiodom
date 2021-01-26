# OptiOdom: An Steering-Agnostic Approach for Odometry Calibration of Wheeled Mobile Robots - GitHub Repository

- [About the Project](#about-the-project)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Data format](#data-format)
  - [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contacts](#contacts)
- [Acknowledgements](#acknowledgements)
- [Funding](#funding)

## About the Project

This repository implements a novel approach to odometry calibration independent of the robot's steering geometry. Also, other methods proposed in the literature are implemented for comparison purposes (as shown in the article elaborated).

## Getting Started

### Prerequisites

- MATLAB R2020

### Data format

- Synchronized odometry and ground-truth data
- CSV files
- See examples in the [data/](https://github.com/sousarbarb/odometry-calibration/tree/main/data) folder

**Metadata file:** `[directory/fileID_metadata.csv]`

```csv
type  , [ diff / tricyc / omni3 / omni4 ],
ngear , [ n ],
encRes, [ Ce ],
Li , [ l (diff/tricyc/omni3) / l1, l2 (omni4) ],
Di , [ D1 , D2 , ... , Dm ],
Thi, [ ThOff (tricyc) ],
N  , [ N (total runs) ],
L  , [ L (square side-length, only needed for UMBmark and Jung&Chung) ],
imarkers,[...], (not required)
gt_ti ,[...],   (not required)
gt_tf ,[...],   (not required)
odo_ti,[...],   (not required)
odo_tf,[...],   (not required)
```

**Data files:** `[directory/fileID_run-[01..N].csv]`

```sh
# Differential drive
[ time (s) ] , [ xgt (m) ] , [ ygt (m) ] , [ thgt (m) ] , [ odoR (ppr) , odoL (ppr) ],
...

# Tricycle
[ time (s) ] , [ xgt (m) ] , [ ygt (m) ] , [ thgt (m) ] , [ odoWh (ppr) , ThWh (rad) ],
...

# Omnidirectional
[ time (s) ] , [ xgt (m) ] , [ ygt (m) ] , [ thgt (m) ] , [ odo1 (ppr) , ..., odoM (ppr) ],
...
```

### Installation

1. Clone the repository

   ```sh
   git clone https://github.com/sousarbarb/odometry-calibration.git
   ```

2. Copy the metadata file and experiments data to the [data/](https://github.com/sousarbarb/odometry-calibration/tree/main/data) folder

3. Execute the odometry calibration method (go to the respective `src` folders for further instructions):

   - OptiOdom: [src](https://github.com/sousarbarb/odometry-calibration/tree/main/src/sousa-et-al), [wiki](https://github.com/sousarbarb/odometry-calibration/wiki/Method)
   - Differential drive-only:
     - Borenstein and Feng - The University of Michigan Benchmark (UMBmark): [src](https://github.com/sousarbarb/odometry-calibration/tree/main/src/diff/umbmark), [wiki](https://github.com/sousarbarb/odometry-calibration/wiki/UMBmark), doi: [10.1109/70.544770](https://doi.org/10.1109/70.544770)
     - Jung and Chung: [src](https://github.com/sousarbarb/odometry-calibration/tree/main/src/diff/jung-and-chung), [wiki](https://github.com/sousarbarb/odometry-calibration/wiki/Jung-and-Chung), doi: [10.1109/ICRA.2012.6224660](https://doi.org/10.1109/ICRA.2012.6224660)
     - Ivanjko et al.: [src](https://github.com/sousarbarb/odometry-calibration/tree/main/src/diff/ivanjko-et-al), [wiki](https://github.com/sousarbarb/odometry-calibration/wiki/Ivanjko-et-al), link: [https://www.researchgate.net/publication/268411270](https://www.researchgate.net/publication/268411270)
   - Ackerman/tricyle-only:
     - Kallasi et al.: [src](https://github.com/sousarbarb/odometry-calibration/tree/main/src/tricyc/kallasi-et-al), [wiki](https://github.com/sousarbarb/odometry-calibration/wiki/Kallasi-et-al), doi: [10.1016/j.robot.2017.04.019](https://doi.org/10.1016/j.robot.2017.04.019)
   - Omnidirectional-only:
     - Lin et al.: [src-omni3](https://github.com/sousarbarb/odometry-calibration/tree/main/src/omni3/lin-et-al) / [src-omni4](https://github.com/sousarbarb/odometry-calibration/tree/main/src/omni4/lin-et-al), [wiki](https://github.com/sousarbarb/odometry-calibration/wiki/Lin-et-al), doi: [10.1109/ICCSE.2019.8845402](https://doi.org/10.1109/ICCSE.2019.8845402)

See [Documentation](https://github.com/sousarbarb/odometry-calibration/wiki) for further details.

## Usage

Example for a differential drive robot using the calibrated kinematic parameters computed by the proposed method:

![diff-arbitrary](https://user-images.githubusercontent.com/36474064/105555069-89e80700-5d00-11eb-858b-2d4779ce4122.png)

Example for a tricycle robot using the calibrated kinematic parameters computed by the proposed method:

![tricyc-arbitrary](https://user-images.githubusercontent.com/36474064/105555068-89e80700-5d00-11eb-8021-2af2f1bf9003.png)

Example for a three-wheeled omnidirectional robot using the calibrated kinematic parameters computed by the proposed method:

![omni3-arbitrary](https://user-images.githubusercontent.com/36474064/105555066-894f7080-5d00-11eb-846b-8c676c5aec3f.png)

## Contributing

Any contributions that you make are greatly appreciated. Please proceed as follows:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/NameFeature`)
3. Commit your changes (`git commit -m 'Add some NameFeature'`)
4. Push to the branch (`git push origin feature/NameFeature`)
5. Open a pull request

## License

Distributed under the [MIT License](https://choosealicense.com/licenses/mit/). See `LICENSE` for more information.

## Contacts

If you have any questions or you want to know more about the work developed by us, please contact one of the contributors of this project:

- Ricardo B. Sousa: ricardo.b.sousa@inesctec.pt _(corresponding author)_
- Marcelo R. Petry: marcelo.petry@inesctec.pt
- Professor António Paulo Moreira: amoreira@fe.up.pt
- Professor Paulo Costa: paco@fe.up.pt

Project Link: <https://github.com/sousarbarb/odometry-calibration>

## Acknowledgements

- [CRIIS - Centre for Robotics in Industry and Intelligent Systems](https://criis.inesctec.pt/) from [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/WEB_PAGE.INICIAL)
- [Universidade de Trás-os-Montes e Alto Douro (UTAD)](https://www.utad.pt/en/)

## Funding

This work is financed by the ERDF - European Regional Development Fund through the Operational Programme for Competitiveness and Internationalisation - COMPETE 2020 Programme, and by National Funds through the Portuguese funding agency, FCT - Fundação para a Ciência e a Tecnologia, within project SAICTPAC/0034/2015- POCI-01-0145-FEDER-016418. This work is also financed by the ERDF - European Regional Development Fund through the Operational Programme for Competitiveness and Internationalisation - COMPETE 2020 Programme within project POCI-01-0145-FEDER-006961, and by National Funds through the FCT - Fundação para a Ciência e a Tecnologia (Portuguese Foundation for Science and Technology) as part of project  UID/EEA/50014/2013.
