# QTrobot-Project
 QTrobot is a humanoid robot built by LuxAI S.A. It is currently in use for the emotional training of children with autism, post-stroke rehabilitation and rehabilitation cognitive and physical of the elderly
 
 We are a team of 3 who have worked on a project that aims to make the behavior of QTrobot more natural. The project therefore aims to provide the robot with a more lively behavior even when he has no work to do. Based on the behavior of a sociable person, it is necessary to make the robot more efficient in terms of interaction with its user and reduce its robotic character. 
 
 For starters, it would be interesting if the robot moves it head and arms a little, like a standing person who is looking at his surroundings. This what the script "Random_moves" aims to do.
Then, The script "Follow_user.py" makes the robot capable of following its user, it was programmed based on the image recognition functions already implemented in its camera NuiTrack. You can see in the image bellow that both motors of the head (HeadYaw and HeadPitch) are fonctionning in order to follow the user's face position
![Suivi ](https://user-images.githubusercontent.com/107966957/176801915-b1261cbb-c0a2-4be2-a63b-7d44f0bc9b75.png)

In addition, scripts "Bonjour.py" and "Namasté.py" helps the robot recognize and reproduce some greetings, in particular the waving, a greeting with a swipe up and the Namasté greeting. In these images you can see some gestures done and the response of the robot to them :
Swipe up / Waving :
![Salut Yudong](https://user-images.githubusercontent.com/107966957/176802330-ca02dbd0-61ac-4743-baa9-25dc15a3f40a.png)
Response :
![Salut à main levée](https://user-images.githubusercontent.com/107966957/176802378-a40276e1-e6c5-4b05-a16a-2416a2bbd4d8.png) : 
Namasté :
![Namasté divya](https://user-images.githubusercontent.com/107966957/176802420-57b151ac-a972-4988-af37-2330191698aa.png)
Response : 
![Namasté](https://user-images.githubusercontent.com/107966957/176802436-448516a4-4d97-40c9-abbd-4c9bacb86e3f.png)


 Finally, bringing together all the functions described previously in a single script, all the fruit of this work can be observed during the same scenario using the script "F_integrale.py". We've posted a video of demonstration of all functions on Youtube. Here's the link to the video : https://youtu.be/oN5SPTHQHew
 
Launching a function :
 
  You should download the pythoncode of the function in the folder catkin~ws of QTPC. Then you just need to open a terminal and tap the command :
  rosrun Name_of_the_function.py (for example rosrun Random_moves.py for the first function). If you've decided to put all scripts in a new folder in the folder catkin~ws you should tap the command :
  rosrun Name_of_the_folder Name_of_the_function.py
  If the script is succesfully launched, a message should appear on the terminal indicating the beginning of the function. See below an exemple of the launch of the first function.
![Capture_terminal](https://user-images.githubusercontent.com/107966957/176799664-758a9d1d-c7b4-4571-87a2-1b5db6880846.png)

In the beginning of each script there are some parameters that you can modify if you want to see the impact of those parameters on the robot's bahavior and this could also help you understand how each function works. 


Members of the project developpers :

Divya RAMDOO
Yudong LIU
Aymane JEBARI
