# Source Code Tugas Besar EL3014 - Sistem Mikroprosesor

## Anggota
- Sidartha Prastya. P   (13219033)
- Kelvin Sutirta        (13219069)

## Deskripsi
Dibuat suatu simulasi PID posisi digital dengan menggunakan Arduino dan LPF (sebagai pengganti motor).
Program dibuat menggunakan ROS dengan 2 devais, yaitu Arduino dan PC.
Arduino berperan sebagai kontroler PID dan PC berperan sebagai motor/LPF.

## Pre-Requisites:
Terdapat beberapa hal yang perlu diperhatikan untuk menjalankan program ini:
- ROS (pembuat menggunakan distro Noetic)
- Rosserial
- Rosserial Arduino

```
Note: Silakan atur hal-hal yang disebutkan di atas terlebih dahulu sebelum menjalankan program.
```
Referensi:
- http://wiki.ros.org/ROS/Tutorials
- https://github.com/ros-drivers/rosserial
- http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup


## Command untuk menjalankan program

> Note: Silakan terlebih dahulu upload file .ino ke Arduino/ESP32 Anda.

```
$ roslaunch motor_sim_lpf motor_sim_lpf.launch
```

- Kebetulan Arduino yang digunakan berada pada port /ttyACM0. Silakan sesuaikan dengan kondisi Anda.
- Silakan atur rqt_plot terhadap sumbu x dan y yang diinginkan.

