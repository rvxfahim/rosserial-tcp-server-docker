#!/bin/bash

# Path to the named pipe
PIPE_FILE="/rosstart/start-roscore.pipe"

# Text to search for
SEARCH_TEXT="started core service [/rosout]"

# Read from the pipe
while read line; do
    if [[ "$line" == *"$SEARCH_TEXT"* ]]; then
        # If the text is found, start rosserial
        supervisorctl start rosserial
        break
    fi
done < "$PIPE_FILE"
