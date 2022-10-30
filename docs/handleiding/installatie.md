
> **note** Tijdens de installatie wordt er vanuit gegaan dat de [ros2 - foxy fitzroy installatie](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) succesvol is doorlopen.

> **note** De installatie is opgezet voor Ubuntu 20.04 focal fossa.

1. Opzetten van de workspace

```bash
source /opt/ros/foxy/setup.bash
mkdir -p ~/wor_sim_review/
```

2. Downloaden van het project

```bash
cd ~/wor_sim_review/
git clone git@github.com:LukevLuijn/wor-simulation.git
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
```

3. Bouwen van de applicatie

```bash
cd ~/wor_sim_review/
colcon build
```

### De applicatie gebruiken met launch script

1. Launch script (new terminal)

```bash
cd ~/wor_sim_review/
./launch.sh
```
### De applicatie gebruiken zonder launch script

1. Applicatie (new terminal)

```bash
cd ~/wor_sim_review/
. install/setup.bash
ros2 launch robot_simulation robot.launch.py
```

2. Rviz (new terminal)

```bash
cd ~/wor_sim_review/
. install/setup.bash
ros2 launch robot_simulation rviz.launch.py
```

3. RQT (new terminal)
```bash
rqt --perspective-file ~/wor_sim_review/src/robot_simulation/config/rqt_config.perspective
```