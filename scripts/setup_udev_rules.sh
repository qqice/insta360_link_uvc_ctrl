#!/bin/bash -e

# USAGE:
# Normal usecase - without any parameters.
#
# [optional parameters]:
# --uninstall : remove permissions for Insta360Link devices.

install=true

for var in "$@"
do
    if [ "$var" = "--uninstall" ]; then
        install=false
    fi
done

if [ "$install" = true ]; then
    echo "Setting-up permissions for Insta360Link devices"
else
    echo "Remove permissions for Insta360Link devices"
fi

exec 3>&2
exec 2> /dev/null
con_dev=$(ls /dev/video* | wc -l)
exec 2>&3

if [ "$install" = true ]; then
    if [ $con_dev -ne 0 ];
    then
        echo -e "\e[32m"
        read -p "Remove all Insta360Link cameras attached. Hit any key when ready"
        echo -e "\e[0m"
    fi
    sudo cp 99-insta360.rules /etc/udev/rules.d/
else
    sudo rm /etc/udev/rules.d/99-insta360.rules
fi

sudo udevadm control --reload-rules && udevadm trigger
if [ "$install" = true ]; then
    echo "udev-rules successfully installed"
else
    echo "udev-rules successfully uninstalled"
fi