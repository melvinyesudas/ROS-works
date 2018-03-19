#!/bin/bash

SUBSYSTEM=="tty", ATTRS(idVendor}=="0403", ATTRS{idProduct)=="6015", 
ATTRS(serial)=="XXXXXXXX",   #it vary .in my case its F002K0
SYMLINK+="sweep"
