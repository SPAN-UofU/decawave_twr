#!/bin/bash

if [ "$1" == grab ]; then
    sudo rm -rf pru/
    sudo cp -r /media/teal/rootfs/root/anh/pru .
    sudo chown -R teal:teal pru/
else
    sudo cp -r pru/ /media/teal/rootfs/root/anh
fi
