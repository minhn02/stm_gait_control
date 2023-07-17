#!/bin/bash
#sshpass -p str ssh -t pi@rover.wlan "echo str | sudo -S date -s @`(date -u +"%s.%N")`"
sshpass -p str ssh -t pi@rover.wlan "echo str | sudo -S date -s @`(echo "$(date -u +"%s.%N") + 1.0" | bc)`"