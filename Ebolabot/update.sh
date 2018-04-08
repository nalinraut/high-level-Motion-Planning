#!/bin/bash

#update Klampt
cd ../../Klampt/
./update.sh

#update SSPP
cd ../iml-internal/SSPP
git pull
make
cd Python
sudo python setup.py install

#remake this
cd ../Ebolabot
make
