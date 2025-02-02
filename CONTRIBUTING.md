# CONTRIBUTING

These guidelines are primarily for new students to understand the Git process and how to start contributing to the repository. Following these guidelines helps prevent poor code practices and streamlines the code-to-robot process. It also allows us to program and review code efficiently, using a method that builds high-quality code and fosters collaboration.

##### Table of Contents  
- [CONTRIBUTING](#contributing)
  * [Issues](#issues)
    + [How to report a bug](#how-to-report-a-bug)
    + [How to suggest a feature](#how-to-suggest-a-feature)
  * [Code Review Process](#code-review-process)
  * [Conventions](#conventions)
    + [Commit Messages](#commit-messages)
    + [Branches](#branches)
      - [Features `feat/`](#features--feat--)
      - [Bug Fixes `bugfix/`](#bug-fixes--bugfix--)
      - [Hot Fixes `hotfix/`](#hot-fixes--hotfix--)
    + [Core Branches](#core-branches)
      - [Main `main`](#main--main-)
      - [Releases `releases/v1.0.0`](#releases--releases-v100-)
    + [Competition Branches](#competition-branches)
    + [Development Branches](#development-branches)
      - [Development `dev`](#development--dev-)
      - [Competition Development `gainesville/dev` `gwinnett/dev`](#competition-development--gainesville-dev---gwinnett-dev-)
  * [Workflow](#workflow)
  * [Contact](#contact)

## Issues

[Click here to create an issue](https://github.com/Chargers-4189/Reefscape-2025/issues/new/choose)

### How to report a bug

Found a bug in the code, or something won't work correctly on the robot?

Fill out an issue and use the `Bug report` template. Ensure you report the bug in detail to make it easier to debug.

[Click here to create a bug report](https://github.com/Chargers-4189/Reefscape-2025/issues/new?template=bug_report.md)

Creating an issue will alert others to the bug, allowing them to offer insight or possible solutions. Once a fix is found, you can create a branch based on development or the branch you're working on. Use the prefix `bugfix/` or `hotfix/`; refer to the branch naming convention for more information.

### How to suggest a feature

Want to suggest a new feature or function on the robot?

Fill out an issue and use the `Feature request` template.

[Click here to create a feature request](https://github.com/Chargers-4189/Reefscape-2025/issues/new?template=feature_request.md)

Feature requests are low priority until all robot functions are programmed correctly. Depending on the feature, it may be pushed to the next version release of the robot. 

## Code Review Process

Code reviews are done via pull requests into the development, main, competition, or release branches. These ensure that the code merging does not have prominent errors or bugs. Then, developers on the team approve the code or request changes. Adhering to the proper conventions will generally streamline this process.

> [!TIP]
> Request reviews from the officers: @Aviator2276 @JDTheBomb @Emm1280

## Conventions

### Commit Messages

Keep the name clear and concise, accurately describing the topic. Limit the length to around 50 characters or less. You can use the description for more informative messages. Use present tense and an imperative mood.

### Branches

Keep the name clear and concise, accurately describing the topic. Limit the length to around 30 characters or less. It must also be lowercase and hyphen-separated. Lastly, always use a prefix to denote the topic.

> [!TIP]
>  When naming your branch, write the full path to the last level. Never start or end a branch with `/`

Here are the prefixes you can use to denote the topic of your branch.
#### Features `feat/`
These branches are used to develop new features and encompass all active development. When creating a new feature, branch off from `dev` to keep the latest code.

- Feature name must be at least 3 characters long
- Feature name must be hyphen-seperated and lowercase

E.g. `feat/swervedrive-auto`

#### Bug Fixes `bugfix/`
Bug fixes address relatively minor issues. When creating a bug fix, create an issue first to document it, then branch off from `dev` or the branch you're working on. Fix the problem and create a pull request to merge it back to the desired branch.

- Bug name must be at least 3 characters long
- Bug name must be hyphen-seperated and lowercase

E.g. `bugfix/swerve-heading-issue`

#### Hot Fixes `hotfix/`
Hotfixes address significant issues; these are emergencies that can be made directly on `core` branches. When creating a hotfix, create an issue first to document it, then branch off from a `core` branch. Fix the problem and create a pull request to merge it back to the `core` branch. This prefix is designated for bugs that can break the robot or pose safety hazards.

- Hotfix name must be at least 3 characters long
- Hotfix name must be hyphen-seperated and lowercase

E.g. `hotfix/motor-burnout-issue`

### Core Branches

> [!WARNING]
>  No direct commits are allowed on these branches. Pull requests are required for merging code.

#### Main `main`
The main branch contains the most stable code regardless of the robot version release. Since this code must be stable, two code review approvals are required before a pull request can be merged. Other than hotfixes, no direct commits can be made on this branch.

#### Releases `releases/v1.0.0`
The releases branch contains the most stable code for a specific robot version release. A new version will be released following mechanical changes. Any new mechanisms on the robot will represent a new version. Two code review approvals are required before a pull request can be merged. Other than hotfixes, no direct commits can be made on this branch.

This branch follows the [Semantic Versioning 2.0.0](https://semver.org/) format, representing versions as X.Y.Z. Here, X represents breaking changes that introduce significant mechanical changes, Y indicates when new features are added, and Z denotes bug fixes that don't introduce features.

### Competition Branches
`gainesville/main` `gwinnett/main`

These competition's main branches contain the most stable code for a specific competition. We separate these branches to differentiate between the competition and current codes. These also require two code review approvals before merging. Again, other than hotfixes, no direct commits can be made on this branch.

### Development Branches

#### Development `dev`
The development branch contains the latest but unstable code. This is where all the feature and bug branches merge before going into the `core` branches. `dev` is the staging point between active development and releases. One code review approval is required; no direct commits besides hotfixes are allowed.

> [!IMPORTANT]
> The development branch will not be used during competitions. There are specific branches for each competition.

#### Competition Development `gainesville/dev` `gwinnett/dev`
These branches contain the latest but unstable code for a specific competition. Unlike the development branch, this does not require code review approval due to how chaotic competition's can be, and direct commits can be made on this branch. Afterward, we can sort out these commits to resolve any errors or bugs on the competition's main branches. 

## Workflow
Before you jump in and start contributing, it’s a good idea to get familiar with the Git Workflow. The power of GitHub ensures that the version history is maintained and multiple versions are retained. Likewise, the code is checked for errors with pull requests to prevent further problems. This allows us to build high-quality code and collaborate efficiently. If you would like to learn more, read [here](https://github.com/Chargers-4189/git-lesson/blob/main/github-course.md#workflow)

## Contact
As always, please email or Discord us if you need help or have a question.

Have fun and happy hacking!