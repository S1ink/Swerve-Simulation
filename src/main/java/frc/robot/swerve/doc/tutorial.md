# Swerve Framework
---
#### This tutorial outlines the swerve control pipeline and provides a minimal implementation of the framework for driving.

## Framework Architecture
The core of the framework is the `SwerveDrive<>` class which is generalized by a specific swerve module type extending the abstract class `SwerveModule`. This class provides a kinematics and odometry solution as well as default logging of target and current sensor states. In most use cases, this class can provide all required functionality for driving and tracking the robot chassis, but can also be extended for extra functionality. This is where the generalized module type comes in the most handy since all type-specific properties of the module implementation will be available to the extended class if needed.

The kinematics and various state container classes differ from standard WPILib implementations in that they allow for full support of second-order (wheel acclerations and steering angular velocities) state propegation. Currently these features are solely future-proofing measures as they are not utilized for the provided driving or tracking helpers. Additionally, most methods or containers that have close WPILib derivatives allow for each conversion or use of WPILib container classes.

Below is a table of the main classes, their usecases, and their locations. **In a minimal swerve usecase, only the top 3 classes need to be accessed in user code (and even more, `ChassisStates` can be exchanged for WPILib's `ChassisSpeeds`)**

| Class Name | Usage | Import Path | File |
| - | - | - | - |
| `SwerveDrive<>` | Generalized off of a `SwerveModule` implementation. Allows for driving, tracking, logging, and *simulation of a swerve drive. | `*.swerve.SwerveDrive` | SwerveDrive.java |
| `SwerveModule` | The base class for all swerve module implementations. Contains functionality for setting state, controlling state, getting sensor data, and optionally simulating. | `*.swerve.SwerveModule` | SwerveModule.java |
| `ChassisStates` | Similar to WPILib `ChassisSpeeds` but also contains second order properties. | `*.swerve.SwerveUtils.ChassisStates` | SwerveUtils.java |
| `SwerveModuleStates` | Combines the WPILib containers `SwerveModulePosition` and `SwerveModuleState` into a single container, with additional second order properties. | `*.swerve.SwerveUtils.SwerveModulePosition` | SwerveUtils.java |
| `SwerveKinematics` | Provides conversions between `ChassisStates` and an array of `SwerveModuleStates`. Implements second-order swerve kinematics. | `*.swerve.SwerveKinematics` | SwerveKinematics.java |
| `SwerveOdometry` | Allows for tracking of the robot's pose based on swerve module sensor data. Internally this class is almost exactly the same as WPILib's `SwerveDriveOdometry` but integrates closely with `SwerveDrive<>` for simpler usage. This class is used internally by `SwerveDrive<>` and normally should not need to be interacted with. External measurement support (vision) is not included but may be added in the future. | `*.swerve.SwerveOdometry` | SwerveOdometry.java |
| `SwerveSimulator` | Used to simulate a swerve drive. Using this class requires an additional implementation of the interface `SwerveModuleModel` for property calculations. _**Currently not functional**_. | `*.swerve.SwerveSimulator` | SwerveSimulator.java |
| `SwerveVisualization` | Allows for conversion of a set of `SwerveModuleStates` to a set of 3d poses for use in AdvantageScope component visualization. Also contains static helpers for creating 2d swerve vector telemetry data. | `*.swerve.SwerveUtils.SwerveVisualization` | SwerveUtils.java |

## Driving Overview
To drive a swerve robot, we need a pipeline that takes user input and commands each swerve module to the required state for driving. We will first break down the high level steps in this pipeline and define how and where each step fits into the framework:

1. Obtain a target state for the entire robot chassis. This is applicable for both driving and trajectory following and can be futher broken down depending on the context.
**For driving:**
    - Gather inputs for **x-speed**, **y-speed**, and **rotational speed**.
    - Apply a deadband if applicable.
    - Apply power scaling if applicable.
    - Convert values to be in physical units of velocity. In practice this means multiplying by the maximum expected velocity.
    - Pre-normalize velocities according to the maximum possible module velocity and maximum allowed module velocity (optional simplification of traditional "post-normalization" step).
    - Rate limit the target state to enforce a maximum accleration.
    - Convert the target state to be of constant curvature.
2. Convert the target state for the entire robot to target states for each module using inverse kinematics.
3. Post-normalize each module's target wheel velocity based on a maximum achievable or allowed wheel velocity.
4. Optimize the rotation of each module to minimize the change in angle required.
5. Apply each module's respective state.

It should be noted that over half of the steps above could be optionally implemented and the driving pipeline would be still functional. The minimally required pipeline includes the steps:

1. Obtain a target robot state...
    - Gather inputs for **x-speed**, **y-speed**, and **rotational speed**.
    - Convert to physical units of velocity.
2. Convert to module states.
3. Apply each module's respective state.
