<p align="center">
<img src=Figure2.png width=100% height=100%>
</p>

# DT-Interface
CESMII Project: Interface for bi-directional communication between FANUC and desktop PC. This project aims for building an intuitive interfance for users with limited experience of customized robot programming language (e.g., KAREL) to quickly plan, analyze, and verify robotic automation such as machine tending and palletization. 

**This repository only contains MuJoCo simulation environment of the SM Framework. To fully build the framework, users require to download [IK-MPC](https://github.com/purduelamm/IK-MPC) AND SM_Framework.**

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

## Table of Contents

- [Repository Structure](#repository-structure)
- [Download Process](#download-process)
- [How to Run](#how-to-run)
- [ToDo Lists](#todo-lists)

## Repository Structure

    ├── crx_10ial_description       # URDF of CRX-10iA/L
    ├── demo                        # main programs to run DT
    ├── manipulator_mujoco          # main MuJoCo environments
    ├── MPC                         # copy from IK-MPC 
    ├── setup.py
    └── trajectory_following_num.py # copy from IK-MPC    

## Download Process

> [!NOTE]
This repository has been tested on [Windows 11](https://www.microsoft.com/en-us/software-download/windows11) and [Ubuntu 22.04](https://releases.ubuntu.com/jammy/).

    git https://github.com/purduelamm/DT-Interface.git
    cd DT-Interface/
    pip3 install -r requirements.txt

## How to Run

Inside demo folder,

    python3 crx10ial_demo.py

For more information about how the environment is set up, please refer to `crx10ial_env.py` in manipulator_mujoco/envs

## ToDo Lists

| **Documentation** | ![Progress](https://geps.dev/progress/60) |
| --- | --- |
