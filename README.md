# balancingRobot
This robot runs on the Elegoo Tumbller robot using an Arduino Every (which comes attached to the robot, although that one is preprogrammed while I started with a new board that wasn't programmed).
Once you have the proper hardware:

(1) Getting all necessary code
Go to the Elegoo Tumbller website: https://www.elegoo.com/blogs/arduino-projects/elegoo-tumbller-self-balancing-robot-car-tutorial?srsltid=AfmBOopgbdQxv_OFNmgNUKrvVvLL0NvCh86JREwgethnexm7QSVZn2xc where all the necessary libraries are available under their **V1.1** links (you can pick either link 1 or link 2, they contain the same code).
Finally, download the three files in this github Repo.

(2) Getting it running on Arduino
Install Arduino 2.3.2 on your computer.
  - If you try an older version (below 2.0), which is what I had, the Elegoo libraries will not work.

Move the Elegoo libraries plus the code from this repo into Arduino within a special file for the robot (not necessary but better readability and safer if you have other projects in Arduino - for example, mine had errors at first because it kept trying to use the libraries I'd downloaded for a different class instead of the Elegoo ones).
  - The Elegoo libraries and the github code should be two subfolders of the robot

Turn on the battery for the robot.
Open the balancing_robot.ino code and compile it (the checkmark in the upper left corner). Click the arrow to send it to the (connected via cable) robot.
Press 'g' and the program will start working (from there you can press the specified letters to get teloperation).
