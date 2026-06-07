* `git clone https://github.com/bxirobotics/bxi_pci_drv.git`
* `cd bxi_pci_drv`
* `make`
* `sudo ./build/motor_test --power --bus 0 --id 1` to enter the interactive motor terminal
* `sudo ./build/motor_test --power --bus 0 --id 1 ping` for a safe motor communication test
* `sudo ./build/motor_test --bus 0 --id 1 listen` to print motor feedback
* `sudo ./build/motor_test --bus 0 --id 1 enable` to enter MIT motor mode
* `sudo ./build/motor_test --bus 0 --id 1 disable` to exit MIT motor mode
* `sudo ./build/motor_test --bus 0 --id 1 cmd 0 0 0 0 0 1000 10` to send MIT command frames for 1 second
* `sudo ./build/motor_test --help` for all motor communication options
* `sudo ./build/drv_test` for drv test
