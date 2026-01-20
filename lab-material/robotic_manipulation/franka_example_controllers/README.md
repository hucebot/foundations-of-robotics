# Custom Franka Controllers

## Description

This README is a tutorial for creating a custom controller by yourself.

## Create your own controller Step by Step

1. Implement the header file in `/include/franka_example_controllers` as `<name_your_controller>.hpp` and source file in `/src` as `<name_your_controller>.cpp`. Templates of both header file and cpp file are created, please refer to the templates for more detailed notice on implementation.

2. Edit `CMakeLists.txt`. Add your controller in `add_library`:
  
      ```CMake
      add_library(
              ${PROJECT_NAME}
              SHARED
              src/joint_impedance_controller.cpp
              src/<name_your_controller>.cpp
      )
      ```

      Edit `franka_example_controller.xml`. Add your controller under `<library path="franka_example_controllers">...</library>`:

      ```xml
      <class name="franka_example_controllers/<NameYourController>"
                type="franka_example_controllers::<NameYourController>" base_class_type="controller_interface::ControllerInterface">
      <description>
          Description of your controller...
      </description>
      </class>
      ```

    Try to build the `franka_example_controllers` pkg with `colcon build --packages-select franka_example_controllers` after you finish the implementation. If there is no problem, let's move on to the last step :)

3. Load necessary parameters and register your controller at `rqt_controller_manager`. Go to `franka_bringup` pkg. Under `/config`, add your controller into `single_controllers.yaml`/`single_sim_controller.yaml`. Choose the yaml file you need according to the launch file your are using (can be found in `launch` under `franka_bringup`).

    Register your controller at controller manager:

    ```yaml
    controller_manager:
      ros__parameters:
        update_rate: 1000  # Hz
        
        joint_impedance_controller:
          type: franka_example_controllers/JointImpedanceController
          
        <name_your_controller>:
          type: franka_example_controllers/<NameYourController>
    ```

    Then edit necessary parameters to be loaded under like:

    ```yaml
    <name_your_controller>:
      ros__parameters:
        arm_id: panda
        <parameter_1>: <xxx>
        <parameters_2>:
          - <xxx>
          - <xxx>
    ```

4. You have finished customizing your controller. Use `franka_bringup` launch to start and then run the `rqt_controller_manager` with `ros2 run rqt_controller_manager rqt_controller_manager`. You should be able see your custom controller in the controller list.
