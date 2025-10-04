(link to my 5 runs: https://drive.google.com/drive/folders/1jfliHHCdIAsk-SeWGHOGdmTrUH1uRUKI?usp=sharing)

# **Thought Process**

## **Hardware struggles**
My primary laptop is a M1 Macbook and my Mother's windows laptop basically gave up on me, so I got a new laptop - Predator Helios 16.

## **Fundamentals**
I started off thinking I could vibe code this, so I tried copiloting the code. It was terrible. I tried to create folders but realized that I didn’t even know how to prepare the setup.py files.

I turned to YouTube to complete the 11-part series where I learned the fundamentals of ROS2 through turtlesim (link: https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&si=Ct3PAETS_9p0FuhE).

## **Dealing with terminals**
I am used to pressing the triangle button in VSCode to run the program, and I found the commands pretty daunting, so I prepared a Notion file to store the commonly used commands (colcon build, source, TCP endpoint, etc.).

Though I eventually got familiar with them, I found it very inefficient to open 6 terminals and rerun each of them. So I decided to create a launch file that runs each program one after another, sending signals to the next to start (guided → activate depth controller → start aligning → commit).

## **Workflow**
1. Switch to guided mode
2. Sink to fixed depth
3. Spin around to detect gate
4. Creep forward until close enough to the gate
5. Commit and just charge forward

## **Switch to guided mode**
(Skipped, fairly straightforward)

## **Sink to depth**
I used Foxglove to find out the acceptable depth my robot could be at to go under the gate. I figured that anywhere from -1.5 m to -2.0 m is safe. I initially explored with -2.0 m but found that in some simulations, when it spawned near the gate, going too low greatly affected its ability to detect the gate. I felt that -1.8 m was the sweet spot and stuck with it.

I also made adjustments to ensure that the robot hovers around that depth before moving on. I spent about 70% of the time debugging this part.

## **Spinning around to detect gate & creeping forward**
There are 2 modes here: SEARCHING and ALIGNING.

1. SEARCHING: if no gate is detected it will just spin. Once the gate is detected for 3 frames and has stabilized, we switch to ALIGNING mode.
2. ALIGNING: the robot uses yaw to align the center of the gate with the center of the screen. It creeps forward slowly while aligning until the area of the gate takes up 25% of the screen (a signal that it is close enough). It then sends on /gate/commit.

I implemented this with a PD controller on yaw error to make the alignment process smoother instead of adjusting forever.

Biggest problems faced:
- I previously wanted to combine these 2 into one, but realized that when the robot is close enough to the gate it may not be able to detect the gate properly, and it would either start spinning on the spot or crash into a pole thinking it was the gate. I tried implementing different tiers (<15%, <30%, ≥30%) but found it easier to just set it into 2 modes. Another reason for having the second mode is that the robot drifts a lot, and if we decide on the trajectory at different distances, the robot will deviate a lot over a large distance. Aligning until it is relatively near the gate helps reduce this uncertainty.
- Sometimes when adjusting, the robot loses the gate. Since it is in aligning mode, it just waddles (small left-right panning in random directions). This is why a lost_grace timer was created. If the gate is lost, it resets and starts spinning again, allowing it to recover. A timer is needed because the gate flickers, and we don’t want false alarms.

## **Commit and just charge forward**

When signaled to commit, it first rotates slightly to the left. This was hardcoded, but it just works. The robot tends to always be a bit more to the right than I want it to be, so this was a last resort after failing at the previous step. I did simulations and found the sweet spot in terms of how much to turn. Once again, this is hardcoded.

Under commit mode, the robot is supposed to just YOLO forward. If it continued to adjust itself, then it would go in circles below the gate or destroy a post. Some adjustments I made were:

- Adding a downward velocity because it tends to float up if uncontrolled
- Lowering the speed, because when it hit a pole at high speed, its direction got heavily affected

ChatGPT suggested making the commit behavior safer using a timed mode. It also suggested ensuring that the commit signal is only listened to once, so the robot just moves forward and doesn’t look back.

## **Moving forward**
If I were given more time, I would go deeper into understanding how the vision works and replace the hardcoded left turn with a more robust alignment check.

## **Conclusion**
I spent most of the time debugging. I found my source control skills to be really rusty in the process. Many problems appeared along the way and ChatGPT kept suggesting ways to patch them instead of dealing with them head-on.

Running simulations is very different in the sense that they are so unstable, and each step must be stabilized before moving forward. This was my first time working with 3D simulations and implementing PD control, and I thoroughly enjoyed the process.











