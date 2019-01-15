# 2019-Robot
Code for Team 670's 2019 "Destination: Deep Space" FIRST Robotics Competition Robot (yet unnamed).

## Repository Organization:<br/>
- 2019-Robot: Java code and files to be deployed to run on the RoboRIO that are organized and deployed with GradleRIO<br/>
- Raspi: Code and files to be used on a raspberry pi coprocessor<br/>
- LED-Arduino: Arduino code to run on an arduino coprocessor to run LED lights that interface with the RoboRIO<br/>
- GRIP-Pipelines: GRIP Vision Pipeline files being tested

## Branching and Workflow on this Repository<br/>
Please check this document for the team's policy for committing code to GitHub!<br/>
https://docs.google.com/document/d/1vO_dtVTDw3-l0x0BabiAE5C45O6bJlQeLL1Uy9McOcQ/edit?usp=sharing <br/>
**Note that you cannot commit directly to master or dev!**<br/>
This project shall follow the following workflow:<br/>

The master branch is considered the stable branch of this project. It may only be updated via pull request from student developer, and then only with Shaylan's approval.<br/>

The dev branch is the main working branch. It may only be updated by pull request from uncontrolled branches.<br/>

For regular development each developer shall create a "feature branch" this is a branch named in the convention: "feature/name" or "bugfix/name". These are for new features and for bugfixes, respectively.<br/>

When work starts on a new feature, its branch will be made off of the latest version of dev, and all development will occur on the branch. When the feature is considered ready, it will be merged onto the dev branch. When merging, automatic merging, LV Merge tool merging, or simply copying and pasting of code fragments may be necessary.
