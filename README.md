# DNN-SFunction-Motor-Control
Deep Learning-based Reduced-Order Model (ROM) for Inverter State Prediction in Linear Motors (PMLSM) using S-Functions.
# DNN-based Reduced-Order Model for Linear Motor Control

## Overview
This repository contains a research project focused on accelerating the computational efficiency of **Model Predictive Control (MPC)** for **Permanent Magnet Linear Synchronous Motors (PMLSM)**. 

By implementing a **Deep Neural Network (DNN)** as a **Reduced-Order Model (ROM)**, we effectively replaced heavy real-time physical computations with a high-accuracy neural network (83% prediction accuracy).

## Key Technical Features
* **Real-time Integration**: The DNN is implemented via a custom **S-Function** to run seamlessly within the Simulink solver loop.
* **Data-Driven Approach**: Includes scripts for generating reference datasets from physical electromagnetic simulations.
* **Control Strategy**: Validated using a full Simulink model for motor state estimation and inverter switching prediction.

## Repository Structure
* `PMLSM.m`: Physical and electromagnetic motor parameters.
* `Data-collection.m`: Script for generating training datasets from MPC simulations.
* `S_function.m`: MATLAB S-Function containing the trained DNN architecture.
* `DNN_Final_Model.slx`: Complete Simulink environment for real-time validation.

## Research Application
This methodology is directly applicable to complex physical systems where high-fidelity simulations (like CFD or Electromagnetics) need to be replaced by fast surrogate models for real-time optimization.
