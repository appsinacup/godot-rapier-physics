#!/bin/bash

# Detect the operating system
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS: Always use the full path to Godot
    GODOT="/Applications/Godot.app/Contents/MacOS/Godot"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux: Use the 'godot' command assuming it's in the PATH or defined by an alias
    GODOT="godot"
else
    echo "Unsupported OS: $OSTYPE"
    exit 1
fi

# Run the test command
$GODOT --headless --path ./bin2d test.tscn --quit-after 1000
