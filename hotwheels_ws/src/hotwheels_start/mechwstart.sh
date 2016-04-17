#!/bin/bash  
sleep 7 
 rostopic pub /robotsound sound_play/SoundRequest "{sound: -2,command: 2, volume: 1.0, arg: '/home/odroid/Downloads/mechwstartup.ogg', arg2: ''}"

