#!/usr/bin/bash
PORT=${RASA_PORT:-5005}
URL=localhost:$PORT/model/parse
rasa run --enable-api -p $PORT &
echo $URL