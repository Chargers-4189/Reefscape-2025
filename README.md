# Reefscape 2025 Monorepo

![](https://updates.team4189.org/content/images/size/w1000/2025/logo/4189-horizontal-black.png)

### FRC Team 4189 - Chargers

This repository includes all code and resources for the 2025 FRC Season, Reefscape.

##### Table of Contents  
- [Resources](#resources)
- [Setup](#setup)
  - [Requirements](#requirements)
- [Contributing](#contributing)

## Resources

```
assets/
└── AdvantageScopeAssets/
    └── Robot_Chargers2025
design/
src/
└── main/
    ├── deploy/
    │   ├── pathplanner
    │   └── swerve
    └── java/
        └── frc/
            └── robot/
                ├── commands
                └── subsystems
```

`assets`
- The robot assets for the FRC Reefscape season and AdvantageScope.

`design`
- Design and code plans for commands and subsystems

`src/main/deploy`
- Autonomous & optimized paths with PathPlanner
- Swerve drive configuration for YAGSL

`src/main/java/frc/robot`
- Robot code


## Setup
To clone and run the code, you'll need [Git](https://git-scm.com/downloads) or [GitHub Desktop](https://desktop.github.com/download/) and WPILib installed on your computer. Refer to the requirements for a complete list. From the command line:
```bash
# Clone this repository
$ git clone https://github.com/Chargers-4189/Reefscape-2025.git
```

### Requirements
To contribute or run the code, you must have the following installed:
- Install the latest version of [WPILib](https://github.com/wpilibsuite/allwpilib/releases/latest)
- Install the latest version of [FRC Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html)
- Install the [Seamless Prettier For Java 2022](https://marketplace.visualstudio.com/items?itemName=hyperproof.vscode-ext-prettier-java) VSCode Extension

## Contributing

For more information about contributing to this project, please read [`CONTRIBUTING.md`](https://github.com/Chargers-4189/Reefscape-2025/blob/main/CONTRIBUTING.md)