#!/bin/bash

if [ "$1" == "sd" ]; then
	if [ "$3" == grab ]; then
	    sudo rm -rf "$2"
	    sudo cp -r /media/roseline/rootfs/root/anh/"$2" .
	    sudo chown -R roseline:roseline "$2"
	else
	    sudo cp -r "$2" /media/roseline/rootfs/root/anh
	fi
else
	if [ "$3" == grab ]; then
	    sudo rm -rf "$2"
	    sudo cp -r /export/rootfs/root/anh/"$2" .
	    sudo chown -R roseline:roseline "$2"
	else
	    sudo cp -r "$2" /export/rootfs/root/anh
	fi
fi