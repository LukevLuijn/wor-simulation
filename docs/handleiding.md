## Handleiding

In het onderstaande onderdeel wordt onder andere besproken hoe de applicatie geinstalleerd kan worden. Hoe deze applicatie gebruikt kan worden en welke requirements gerealiseerd zijn, en hoe.

### Installatie instructies

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

### Bewegen van de robot

#### Demo script

Er is een kleine demonstratie opgezet om de verschillende capaciteiten van de robot weer te geven.
Dit demo script zal onder andere het kopje oppakken en verplaatsen. Met het onderstaande commando kan het demo script uitgevoerd worden.

1. Demo script (new terminal)

```bash
cd ~/wor_sim_review/
. install/setup.bash
./demo.sh
```

#### Bewegen door middel van commando's

De verschillende servo's in de AL5D robot kunnen bewogen worden door middel van seriele commando's.
Omdat deze applicatie een simulatie is van de daadwerkelijke robotarm moeten de commando's verzonden worden door middel van een publicatie naar een ros topic waar de virtuele controller naar luistert.

De commando's zijn onderverdeeld in drie onderdelen:

```text
Voorbeeld commando: #0P1200S5000

[#0]     start char van het commando, direct opgevolgd door de index van de servo.
[P1200]  'P' char gevolgd door de gewenste PWM waarde voor de servo.
[S5000]  'S' char gevolgd door de gewenste tijd hoelang de beweging moet duren.
```

Commando's voor de verschillende servo's kunnen samengevoegd worden zodat er een synchrone beweging uitgevoerd kan worden.

```text
#0P2500S500#1P1833S500#2P1444S500#3P722S500#4P500S500#5P1000S500
```

Commando's publiceren naar het topic waar de arm naar luistert: ```/sim/controller/command``` kan gerealiseerd worden door middel van het volgende commando:

```bash
# eenmalig
cd ~/wor_sim_review/
. install/setup.bash
```

```bash
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S500#1P1833S500#2P1444S500#3P722S500#4P500S500#5P1000S500'}"
```

Verschillende voorbeelden van bewegingen zijn terug te vinden in het demo script (/demo.sh)


### Requirements

|  #   |  Prio  |      Behaald       | Beschrijving                                                                                                                                                                                                                                                                                                                                                                                                         |
|:----:|:------:|:------------------:|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PA01 | Should | :heavy_check_mark: | De directory structuur beschreven in de ROS2 tutorial ( [creating a package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) ) is aangehouden tijdens de ontwikkeling van het project.                                                                                                                                                                       |
| PA02 |  Must  | :heavy_check_mark: | Het project is getest en geschreven met/voor de colcon build tool gebruikt door ROS2.                                                                                                                                                                                                                                                                                                                                |
| PA03 |  Must  | :heavy_check_mark: | De verschillende classen en andere onderdelen van de code zijn geschreven aan de hand van in de OSM course geleerde OO principes.                                                                                                                                                                                                                                                                                    |
| PA04 | Should | :heavy_check_mark: | De styleguide is voorafgaand aan de ontwikkeling van het project doorgenomen en toegepast tijdens. Verder is de .clang-format gebruikt voor het formateren van de code volgens de door ros bepaalde opmaak ( [.clang-format](https://github.com/ament/ament_lint/blob/26397786f603b8e9e4c3c399c3d33b1c6873ee0d/ament_clang_format/ament_clang_format/configuration/.clang-format) )                                  |
| VS01 |  Must  | :heavy_check_mark: | Zoals beschreven in de handleiding kan de virtuele controller aangestuurd worden door middel van seriele commando's opgezet volgens de in de lynxmotion beschreven handleiding. De ondersteunde commando's zijn: P, positie per servo, S, tijd per positie en stop, (STOP), voor het uitvoeren van een noodstop.                                                                                                     |
| VS02 |  Must  | :heavy_check_mark: | De joint_state berichten van de robotarm worden gepubliceerd op het topic: /joint_states. Deze berichten kunnen worden ingezien door middel van het commando: ```ros2 topic echo /joint_states```                                                                                                                                                                                                                    |
| VS03 |  Must  | :heavy_check_mark: | Door het commando voor het starten van de rviz applicatie (beschreven in de handleiding) uit te voeren is het model van de AL5D robot te zien.                                                                                                                                                                                                                                                                       |
| VS04 |  Must  | :heavy_check_mark: | De verschillende servo's hebben ieder een maximale snelheid meegekregen (gebaseerd op de datasheet per servo). De snelheid wordt verder nog aangepast op basis van de meegegeven tijd in milliseconden voor het uitvoeren van een commando.                                                                                                                                                                          |
| VS05 | Should | :heavy_check_mark: | In het launch document [src/robot_simulation/launch/robot.launch.py](https://github.com/LukevLuijn/wor-simulation/blob/4c382b160dac39e8816105fc62bb3a63c29a7470/src/robot_simulation/launch/robot.launch.py#L13-L14) zijn twee variablen gedeclareerd (robot_pos_x & robot_pos_y) voor het bepalen van de positie van de robotarm. Deze variablen kunnen aangepast worden waardoor de robot zal verplaatsen in rviz. |
| VC01 | Should | :heavy_check_mark: | Zie VS05, de positie van de beker wordt bepaald op basis van de locatie van de robotarm.                                                                                                                                                                                                                                                                                                                             |
| VC02 |  Must  | :heavy_check_mark: | Door middel van een [marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html) wordt er een .stl document ([src/robot_simulation/model/wor_sim_cup.stl](https://github.com/LukevLuijn/wor-simulation/blob/4c382b160dac39e8816105fc62bb3a63c29a7470/src/robot_simulation/model/wor_sim_cup.stl)) van een beker gepubliceerd in Rviz                                                          |
| VC03 | Should |        :x:         | Deze eis is niet gerealiseerd.                                                                                                                                                                                                                                                                                                                                                                                       |
| VC04 | Could  |        :x:         | Deze eis is niet gerealiseerd.                                                                                                                                                                                                                                                                                                                                                                                       |
| VC05 | Should | :heavy_check_mark: | Zodra de robotarm de beker vastpakt veranderd de kleur van de beker van wit (255,255,255) naar cyan (0,255,255), [verwijzing](https://github.com/LukevLuijn/wor-simulation/blob/4c382b160dac39e8816105fc62bb3a63c29a7470/src/robot_simulation/src/cup_node.cpp#L80-L83).                                                                                                                                             |
| VC06 |  Must  | :heavy_check_mark: | Zodra de robotarm de beker vastpakt veranderd de locatie van de beker op basis van het middelpunt van de twee gripper armen, [verwijzing](https://github.com/LukevLuijn/wor-simulation/blob/4c382b160dac39e8816105fc62bb3a63c29a7470/src/robot_simulation/src/cup_node.cpp#L65-L78).                                                                                                                                 |
| VC07 |  Must  | :heavy_check_mark: | Zodra de beker niet vastgehouden word door de robotarm en de beker niet op de grond staat zal er zwaartekracht toegepast worden zodat de beker naar de grond toe valt, [verwijzing](https://github.com/LukevLuijn/wor-simulation/blob/4c382b160dac39e8816105fc62bb3a63c29a7470/src/robot_simulation/src/cup_node.cpp#L90-L101).                                                                                      |
| VC08 |  Must  | :heavy_check_mark: | De virtuele beker publiceert zijn positie naar een topic: ```/sim/cup/pose```. Dit topic kan uitgelezen worden door middel van het commando: ```ros2 topic echo /sim/cup/pose```.                                                                                                                                                                                                                                    |
| VC09 | Should | :heavy_check_mark: | De virtuele beker publiceert zijn snelheid naar een topic: ```/sim/cup/speed```. Dit topic kan uitgelezen worden door middel van het commando: ```ros2 topic echo /sim/cup/speed```.                                                                                                                                                                                                                                 |
| VC10 | Could  | :heavy_check_mark: | De actuele snelheid van de beker (hetzelfde als het topic) kan getoond worden in rqt_plot door middel van het volgende commando: ```rqt --perspective-file ~/wor_sim_review/src/robot_simulation/config/rqt_config.perspective```                                                                                                                                                                                    |
| DI01 |  Must  | :heavy_check_mark: | Er is een demo script opgesteld voor het demonstreren van de verschillende capaciteiten van de applicatie [demo.sh](https://github.com/LukevLuijn/wor-simulation/blob/4c382b160dac39e8816105fc62bb3a63c29a7470/demo.sh)                                                                                                                                                                                              |
| DI02 | Could  | :heavy_check_mark: | Zoals verteld bij eis VS05 is dit onderdeel opgenomen in het launch document.                                                                                                                                                                                                                                                                                                                                        |
| DI03 | Could  | :heavy_check_mark: | Zoals verteld bij eis VC01 is dit onderdeel opgenomen in het launch document.                                                                                                                                                                                                                                                                                                                                        |
| DM01 |  Must  | :heavy_check_mark: | Zie hoofdstuk: Handleiding/Installatie instructies.                                                                                                                                                                                                                                                                                                                                                                  |
| DM02 |  Must  | :heavy_check_mark: | Zie hoofdstuk: Handleiding/Bewegen van de robot.                                                                                                                                                                                                                                                                                                                                                                     |
| DM03 |  Must  | :heavy_check_mark: | Zie hoofdstuk: Handleiding/Requirements.                                                                                                                                                                                                                                                                                                                                                                             |
| DD01 |  Must  |        :x:         | TODO                                                                                                                                                                                                                                                                                                                                                                                                                 |
| DD02 |  Must  |        :x:         | TODO                                                                                                                                                                                                                                                                                                                                                                                                                 |
| DD03 | Could  |        :x:         | TODO                                                                                                                                                                                                                                                                                                                                                                                                                 |
| DD04 | Should |        :x:         | TODO                                                                                                                                                                                                                                                                                                                                                                                                                 |