#!/bin/bash

if [ "$1" == grab ]; then
    sudo rm -rf dw1000_bbb/
    sudo cp -r /export/rootfs/root/anh/dw1000_bbb .
    sudo chown -R teal:teal dw1000_bbb/
else
    sudo cp -r dw1000_bbb/ /export/rootfs/root/anh
fi
