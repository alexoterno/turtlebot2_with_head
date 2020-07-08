<!-- [![Contributors][contributors-shield]][contributors-url] -->
<!-- [![Forks][forks-shield]][forks-url] -->
<!-- [![Stargazers][stars-shield]][stars-url] -->
<!-- [![Issues][issues-shield]][issues-url] -->
<!-- [![MIT License][license-shield]][license-url] -->
<!-- [![LinkedIn][linkedin-shield]][linkedin-url] -->


<!-- PROJECT LOGO -->
<div align="center">
  <a href="https://github.com/alexoterno/turtlebot2_with_head">
    <img src="images/turtlebot2_ari_head.png" alt="Logo" width="160" height="250">
  </a>

  <h3 align="center">turtlebot2_with_head</h3>

  <div align="center">
    This repository contains the turtlebot2 robot on ROS Melodic with the ARI Head
    <br />
    <a href="https://github.com/alexoterno/turtlebot2_with_head"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/alexoterno/turtlebot2_with_head">View Demo</a>
    ·
    <a href="https://github.com/alexoterno/turtlebot2_with_head/issues">Report Bug</a>
    ·
    <a href="https://github.com/alexoterno/turtlebot2_with_head/pulls">Request Feature</a>
  </div>
</div>

<!-- TABLE OF CONTENTS -->
## Table of Contents
* [About the Project](#about-the-project)
  * [Package Description](#package-description)
    * [head_src](#head)
    * [inria_src](#inria)
    * [turtlebot2_src](#turtlebot2)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Roadmap](#roadmap)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)

## About the Project
This repository contains the turtlebot2 robot for ROS Melodic version of ROS with the [ARI head](http://wiki.ros.org/Robots/ARI/Tutorials).
A plate on the top of the basic [turtlebot2](http://wiki.ros.org/Robots/TurtleBot) was added (the third one) to increase the height of the robot. On top of this, the head of the ARI robot was put.

### Package Description

#### head_src/src
This package contains the packages for the ARI Head and the [robotis_op3](http://wiki.ros.org/robotis_op3) head (now, it's deprecated).
Regarding the ARI head packages, the description files can be found in the [ari_description](https://github.com/alexoterno/turtlebot2_with_head/tree/master/src/head_src/src/ari_head/ari_description) package. The [controller_manager](https://github.com/alexoterno/turtlebot2_with_head/tree/master/src/head_src/src/ari_head/controller_manager) and [controller_manager_msgs](https://github.com/alexoterno/turtlebot2_with_head/tree/master/src/head_src/src/ari_head/controller_manager_msgs) packages are used to control the head.

#### inria_src/src

#### turtlebot2_src/src


<!-- GETTING STARTED -->

## Getting Started
To run some packages of this repository project on the two proposed robot simulation, follow these simple steps.

### Prerequisites
In order to run the learning, control and interactions packages with the robot simulation on your machine and given the Ubuntu, ROS and Gazebo versions constraints, the roll-out is split in two. Waiting we solve the container network issue with singularity compose, the simulation will be run on your local machine and this repository project will be run on a container with [singularity](https://sylabs.io/guides/3.5/user-guide/). So, you need to install :
* [ROS Melodic](http://wiki.ros.org/melodic/Installation) and [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu) in your local machine (it can be a virtual machine)
* [Singularity](https://sylabs.io/guides/3.5/admin-guide/installation.html) or [Docker](https://docs.docker.com/engine/install/ubuntu/) container

### Installation


<!-- USAGE EXAMPLES -->
## Usage

<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/alexoterno/turtlebot2_with_head/issues) for a list of proposed features (and known issues).



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

<!-- Distributed under the MIT License. See `LICENSE` for more information. -->



<!-- CONTACT -->
## Contact

Alex Auternaud  - alex.auternaud07@gmail.com<br/>
Project Link: [https://github.com/alexoterno/turtlebot2_with_head](https://github.com/alexoterno/turtlebot2_with_head)<br/>



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* []()
* []()
* []()


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
<!-- [contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=flat-square -->
<!-- [contributors-url]: https://gitlab.inria.fr/perception-ral/ari_spring_project/-/project_members -->
<!-- [forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=flat-square -->
<!-- [forks-url]: https://gitlab.inria.fr/perception-ral/ari_spring_project/-/forks/new -->
<!-- [stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=flat-square -->
<!-- [stars-url]: https://gitlab.inria.fr/perception-ral/ari_spring_project/-/starrers -->
<!-- [issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=flat-square -->
<!-- [issues-url]: https://gitlab.inria.fr/perception-ral/ari_spring_project/-/issues -->
<!-- [license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=flat-square -->
<!-- [license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt -->
<!-- [linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555 -->
<!-- [linkedin-url]: https://linkedin.com/in/othneildrew -->
<!-- [product-screenshot]: images/screenshot.png -->
