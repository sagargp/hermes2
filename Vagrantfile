# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ppg/ros-jade"
  config.vm.network "public_network"
  config.vm.synced_folder "ros-src/hermes", "/home/vagrant/catkin_ws/src/hermes/"

  config.vm.provider :virtualbox do |vb|
    vb.customize ['modifyvm', :id, '--usb', 'on']
    vb.customize ['usbfilter', 'add', '0', '--target', :id, '--name', 'xbox', '--vendorid', '0x045E']
  end
end
