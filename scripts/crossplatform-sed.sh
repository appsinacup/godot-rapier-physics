#!/bin/bash
# From https://gist.github.com/maxpoletaev/4ed25183427a2cd7e57a

case "$OSTYPE" in
    darwin*)  PLATFORM="OSX" ;;
    linux*)   PLATFORM="LINUX" ;;
    bsd*)     PLATFORM="BSD" ;;
    *)        PLATFORM="UNKNOWN" ;;
esac

echo $PLATFORM

if [[ "$PLATFORM" == "OSX" || "$PLATFORM" == "BSD" ]]; then
    sed -i "" "$1" "$2"
elif [ "$PLATFORM" == "LINUX" ]; then
    sed -i "$1" "$2"
fi
