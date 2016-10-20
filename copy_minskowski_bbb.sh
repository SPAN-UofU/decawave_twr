#!/bin/bash

if [ "$1" == grab ]; then
    sudo rm -rf minskowski/
    sudo cp -r /export/rootfs/root/anh/minskowski .
    sudo chown -R teal:teal minskowski/
else
    sudo cp -r minskowski/ /export/rootfs/root/anh
fi
