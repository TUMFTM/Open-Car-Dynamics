# double_track_model

## Usage

```cpp
colcon build --packages-select double_track_model && . install/setup.zsh
ros2 run double_track_model test_node
```

## Run tests
```cpp

colcon build --packages-select double_track_model
. install/setup.zsh
colcon test --packages-select double_track_model
colcon test-result --verbose
```