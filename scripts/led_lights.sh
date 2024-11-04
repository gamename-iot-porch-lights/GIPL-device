#!/bin/bash

aws iot-data publish \
    --profile tennis@gamename \
    --region us-east-1 \
    --endpoint-url "https://a3u37c52vq0b6j-ats.iot.us-east-1.amazonaws.com" \
    --topic "home/porch/lights/illumination" \
    --cli-binary-format raw-in-base64-out \
    --payload "{\"LED\": \"ON\"}"