# Ant_Swarm_BOTs
This interdisciplinary project at the edge of swarm robotics, biomimetism, and odometry is trying to simulate the behavior of worker ants.

# Core Concept

With more than 20,000 species worldwide and a multi continental presence, ants are undeniably a successful genius. These eusocial insects of the family Formicidae appeared in the early Cretaceous and are amazingly diverse. From small colonies of predatory ants that fight for the role of laying queen, to large colonies of millions of coordinated individuals with various castes assigned by birth, ants colonies are often described as super-organisms because the ants appear to operate as a unified entity, collectively working together toward a common goal: support the colony.

# Robotic

This complex organisation has been studied for decades, and inspired the field of swarm robotics.

Even today, robots are built to function on their own. And when we need them to coordinate with one another, we usually tend to use a central intelligence system telling each machine what to do so they don’t interfere with another machine. 

What if we took inspiration from eusocial insects and their collective intelligence to try and make autonomous and independent robots that act collectively as a super-organism?

I decided to focus on this question and to build my own swarm of robots inspired by ants collective social behavior.  

To simulate worker ants foraging behavior, my plan consists of using multiple Zumo bots, and, rather than making them fight, make them work together. The Zumo bots will evolve on a uniformly colored plane (grown). They will leave their home base (ant nest) and forage to search for a marker on the ground (food). Once one individual found the marker (food), just like ants do, they will go back to their nest and leave a pheromone trail behind to inform other ants.

Not all ants use pheromones to communicate with each other. All species of ants rely on eyesight (1,2) and magnetic field (3,4) for orientation. Only large colonies with many castes rely on pheromone trails to communicate interesting places to go to ant-mates. Usually those colonies are composed of sterile, wingless females, most of which are workers (ergates), as well as soldiers (dinergates) and other specialised groups. I initially planned to mimic these types of colony behaviors for my ant bots, however, after encountering issues with the "pheromone" trail, my robots are not able to give information on the position of food marker. So they behave more like small colonies of bull ants, also known as Myrmecia. Bull ants are one of the most primitive group of ants on earth. These workers are omnivorous (generally, worker ants only consumes sweet substance, keeping the protein for the larvae growth) and exhibit greater longevity than other ants. Myrmecia worker ants are also known as gamergate, meaning they are also able to reproduce with male ants, usually fighting for the role of laying queen. Workers are solitary hunters and do not lead other workers to food (no pheromone trail). They rather rely on their excellent vision and earth magnetic field for orientation.


# My robot: Ant.S

For my robot's ant orientation in the 3D plane, I decided to use the magnetometer of the Zumo bots as a compass to have the robot move randomly based on the magnetic field. I am also implementing an ultrasonic sensor to give sight to each ant bots for collision detection. An IR receptor is present on the ants to find the home (IR transmitter).
I soldered to the zumo extra pins so I could connect the ultrasonic sensor Vcc to 5V, Gnd to Gnd, trigpin to 4, and echopin to 1 on the Arduino card. To connect the IR receptor to find the ant nest, I connected the output pin to A0.

I used the zumo's reflectance sensor array to detect the food marker which is composed of both an object surrounded by black ground. So the robots identify the object as food only if it sees both a black line and an object in front of him. After a short pose and time to "eat", he will turn around and start looking for the IR beacon. Once he found the beacon he will start to head home.

I decided to make the food marker by using both the ultrasonic sensor and the zumo's reflectance sensor array in order to use the zumo's reflectance sensor array for a future implementation of line following of the "pheromone" trail.

![image](https://api.projects.cri-paris.org/api/projects/7qrxPKl2/images/61bc736104e1d685876b49a5)

# Materials

- 2 Arduino Leonardo or UNO & ARduino IDE
- Zumo bots for the Ant.S 
    - Use zumo reflectance sensor array to detect the food marker and the “pheromone” trail
    - Use zumo magnetometer as a compass for orientation
- IR transmitter (ant nest) & receiver (find the ant nest)
- 2 Bread boards 
- 9 male/female cables
- Ultrasonic sensor to detect object in front of the ants
- Battery for zumo (AA) & for nest (9V)
- 330 Ohm resistor for nest

# Schematics of robots connections with arduino

![image](https://api.projects.cri-paris.org/api/projects/7qrxPKl2/resources/61bca2db34fd90bcbf6b49cd)

# Schematic of IR transmitter connections

![image](https://api.projects.cri-paris.org/api/projects/7qrxPKl2/resources/61bca3fab9de1252946b4a15)

# Libraries

for IR transmitter:
#include <IRremote.h> // >v3.0.0

for ant bots:
//Libraries for Magnetometer & zumo IR sensors
#include <Wire.h>
#include <ZumoShield.h>

//Libraries for Ultrasonic sensor
#include <ZumoMotors.h>
#include <Pushbutton.h>

//IR receiver
#include <IRremote.h>

# Issues :

Installing Arduino IDE on Ubuntu was a real struggle. I had root issues with the port on Arduino Leonardo. I also tried installing multiple versions of Arduino using apt, snap, the Ubuntu application installer, and directly downloading the link from arduino.com. In the end I was successful in installing Arduino, but I still had to reset the Arduino card any time I was compiling a code. I later realized that I would have had none of these issues using Arduino Uno. This process took me approximately an entire day.

Another issue I had was replacing delay() with millis() in my code.
Millis interferes with magnetometers (delay does not), so to use millis I had to increase the threshold.

For the trail I intended to use a color changing paint such as thermochromic paint and a heater, or a photosensitive paint and a laser. However, after testing both I realise that neither were being detected by the ultrasonic sensor. I also tried seeing if water (wetness) was detected, with no success. I would have needed the trail to be traced with a black marker. However in the end I didn’t have time. 

Another issue I ran into was that for the line following test, the lines I tested using a black marker were too thin for the zumo to pick up for line following. The lines need to be at least 1 cm thick. 

Concerning the IR beacon, I had trouble orientating the receptor to make it unidirectional.

# What's next?

I wasn't able to complete all my initial goals due to technical issues and materials not working as expected (talking about the "pheromone" trail), but I am still very pleased of how far I got. I managed to use a zumo bot to implement some foraging behavior, collision detection and avoidance, food searching, and heading home to brink back the food.

The next step would be to look for a material that would work with zumo's reflectance sensor array for line following, in order to leave a TEMPORARY trail behind the Ant.S after it found the food marker. I already worked on the line following script so that would not be hard to implement it to my code.

I also need to build more bots, or to be precise to solder extra pins to other zumo bots in order to add an ultrasonic sensor and an IR receiver.

# References

[Pololulu Zumo Shield for Arduino User’s Guide](https://www.generationrobots.com/media/user-guide-zumo-robot-for-arduino.pdf) ⇒ Zumo related libraries & sensors

[It Can See; Giving Your Bot Sight!](https://www.intelisecure.com/it-can-see-giving-your-bot-sight/) ⇒ Ultrasonic Sensor

[Compass Calibration and Sampling](https://www.cs.lafayette.edu/~pfaffmaj/courses/s15/cs106l/zumo_robot_labs/lab04/) ⇒ Magnetometer

[Turn with compass](https://www.pololu.com/docs/0J57/7.f) ⇒ Magnetometer

[IR receiver](https://forum.arduino.cc/t/ir-receiver-continuous-signal/391770/5)

[IR transmitter](https://mschoeffler.com/2021/05/01/arduino-tutorial-ir-transmitter-and-ir-receiver-hx-m121-hx-53-ky-005-ky-022-keyes-iduino-open-smart/)

