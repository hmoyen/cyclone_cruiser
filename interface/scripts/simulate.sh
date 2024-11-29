#!/bin/bash

BROKER=localhost
PORT=1883
TOPIC_TRAJECTORY="/trajectory"
TOPIC_SONAR1="sonar1"
TOPIC_SONAR2="sonar2"

# Simulate random trajectory data (X,Y coordinates)
X=0
Y=0
DX=1
DY=1

while true; do
  # Update position (simulated)
  X=$((X + DX))
  Y=$((Y + DY))

  # Simulate sonar readings (random distance in cm)
  SONAR1_DISTANCE=$((RANDOM % 100 + 10))
  SONAR2_DISTANCE=$((RANDOM % 100 + 10))

  # Publish the data to the MQTT topics
 # mosquitto_pub -h $BROKER -p $PORT -t $TOPIC_TRAJECTORY -m "X:$X;Y:$Y"
  mosquitto_pub -h $BROKER -p $PORT -t $TOPIC_SONAR1 -m "D:$SONAR1_DISTANCE"
  mosquitto_pub -h $BROKER -p $PORT -t $TOPIC_SONAR2 -m "D:$SONAR2_DISTANCE"

  # Wait before publishing again
  sleep 1
done
