#!/bin/bash

if [ "$1" == grab ]; then
    sudo rm -rf ant_cal/
    sudo cp -r /export/rootfs/root/anh/ant_cal .
    sudo chown -R teal:teal ant_cal/
else
    sudo cp -r ant_cal/ /export/rootfs/root/anh
fi
