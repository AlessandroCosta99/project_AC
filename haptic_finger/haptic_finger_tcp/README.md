# Haptic-Finger
Get started with Haptic Finger: RasPi configuration + sensor set up

## What do you need
- Raspberry Pi (Any version, in this tutorial I'm using the Zero W)
- Haptic Finger
- micro SD card and adaptor (USB adaptor if your pc doesn't have the SD port)
- cables, connectors and resistors
- Ubuntu (Any version, in this tutorial I'm using Ubuntu 18.04) 
- a stable connection (NOT eduroam). One possibility is using eduroam on your phone and connect your pc and the Pi to your phone hotspot. By doing that you're creating another access to eduroam but managable since you have an ssid and a password created by you.
 
 UPDATE : local ssh , ethernet connection through USB cable, BEST option -> Tutorial https://johnnymatthews.dev/blog/2021-02-06-connect-to-raspberry-pi-zero-over-usb-on-ubuntu/

## Pipeline
This tutorial has the following pipeline:
- Ch.1 Installing the OS on the SD card: if you don't know it yet, the Raspberry Pi is a computer! In order to work requires an operating system (OS)
- Ch.2 Connecting and detecting the Raspberry Pi through your pc: here we're going to do what is called "Headless" configuration. Headless 'cause we're not going to connect monitor or keyboard to the Pi board but you will program on it using your computer
- Ch.3 Installing VNC in order to have a virtual monitor: still headless but for what you're going to do is better to have an IDE and a proper environment to work and visualize stuff and not doing all by terminal.
- Ch.4 Haptic Finger connection: finally some hardware work! Here I'll show the electrical circuit that you have to make to connect in the proper way all the parts that make the haptic finger (camera and led), then how the camera can be detected by the OS and a simple script to visualize something.  


## 1. Raspberry Pi OS
From this website (https://www.raspberrypi.com/software/) you can download Raspberry Pi Imager, the official software that allows you to install the OS on the SD that will be put in the board.

PAY ATTENTION

Not every Imager runs on every version of Ubuntu, if you are using Ubuntu 18.04 the latest Imager available is 1.6.1 and this is the terminal command to download it 

sudo apt install ./imager_1.6.1_amd64.deb

Things to do before writing the OS:
- plug in the SD card
- format just the "boot" partition that will appears on your files manager

After the installation, open Imager by clicking on the Raspberry icon and select the OS (the reccomended one, if your not completely sure on what are you doing) but wait the next step before pressing the write buttom.

There is a secret keyboard sequence that allows you to set the Wi-Fi specifications and the SSH in order to access it later on, so press

ctrl + shift + x

After complete this step you can go back to the previous window and, finally, press write. You'll have to wait some minutes but after it is complete you'll have successfully installed the OS on your SD card, this is the conclusion of this chapter.

## 2 Accessing the Raspberry Pi

Unplug the SD card and put it in the specific port of the Raspberry, now, connect through the cable the board with the pc (micro USB port board side - USB port computer side), the Raspi should starts blinking.
To detect it, be sure that your pc is connected at the same wi-fi, then install nmap by digiting in the terminal window

sudo apt-get install nmap

Now let's find your pc IP address, digit

hostname -I

and save it.

Now we're going to find all the devices that are connected to this wi-fi

nmap -sP <IP address of your pc without the last .number>.0/24

If your connection is local you should detect few devices and finding the Pi should be easy, otherwise try to plug and unplug it and see which IP appears/disappears -> save the IP of the Pi
  
Now we will access the Pi by doing
  
ssh pi@<-IP address->
  
You have to digit the password that you chose previously (Ch.1) and you should be in the RasPi terminal  
  
## 3 VNC
Virtual Network Computing (VNC) is a graphical desktop-sharing system that provides a graphical interface to remotely control another computer.

  Raspberry Pi comes with VNC Server and VNC Viewer installed.
  You will need to enable VNC software on your device, the Pi, before being able to use it. Run the command sudo apt-get update, then sudo apt-get install realvnc-vnc-server.   
  Then, run sudo raspi-config to access to the settings of the raspberry, here, together with other functionalities, you should be able, navigating to the proper section to enable VNC, if it's not already enabled.
  
On your computer download VNC Viewer - https://www.realvnc.com/en/connect/download/viewer/ - and create an account, we're going to use Viewer to visualize the raspberry desktop.
  
Now, on the Pi, run the command - vncserver - to create a virtual desktop and write down the IP address and display number.
  Next, enter it into VNC Viewer, write the IP address associated with the virtual desktop and you will be able to connect, you'll be required to insert the username and the password that you set for the raspberry Pi, default username is Pi.

With the graphical interface will be much easier running scripts and using the Pi like a real computer.  

## 4 Using the Haptic Finger
  
Connect the finger as depicted in figure

![IMG_20230109_222238](https://user-images.githubusercontent.com/63586999/211423784-29a36334-7259-4003-b37e-09b86fb6a99f.jpg)

  Enable the camera by switch on the respective voice in the Pi settings 
  
  Now everything is done, you just need to run a script to start the video streaming from the camera.
  
## Code

client.py should run on the raspberry pi

server.py should run on the main computer

replace "IP_Address" with your actual local IP Address of the raspberry pi

## Useful Material
  
Fast high-res reliable streaming through Python socket - https://raspberrypi.stackexchange.com/questions/73978/can-i-use-the-pi-camera-stream-in-python-on-a-pc  
 
Easy Streaming, just as check that everything is working -  https://randomnerdtutorials.com/video-streaming-with-raspberry-pi-camera/
 
Advanced Recipies -  https://picamera.readthedocs.io/en/release-1.13/recipes2.html#rapid-capture-and-streaming
 
Headless setup - https://linuxhint.com/raspberry_pi_headless_mode_ubuntu/
 
Detailed description of different steps and settings - https://learn.sparkfun.com/tutorials/python-programming-tutorial-getting-started-with-the-raspberry-pi/all 
