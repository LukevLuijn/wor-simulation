
# 1. Handleiding <a name="chapter0"></a>

In de onderstaande hoofdstukken zullen twee aspecten aan bod komen. Ten eerste de installatie instructies. Deze instrcuties vertellen stap voor stap hoe de applicatie gebouwd kan worden.

Het tweede onderdeel; 'Bewegen van de robot', zal uitlegen hoe de applicatie gebruikt kan worden, dat wil zeggen, hoe de gebruiker de robot kan aansturen.

# Inhoudsopgaven

- 1 [Handleiding](#chapter0)
	- 1.1 [Installatie instructies](#chapter1)
		- 1.1.1 [De applicatie gebruiken met launch script](#chapter2)
		- 1.1.2 [De applicatie gebruiken zonder launch script](#chapter3)
	- 1.2 [Bewegen van de robot](#chapter4)
		- 1.2.1 [Demo script](#chapter5)
		- 1.2.2 [Bewegen door middel van commando's](#chapter6)

<div style="page-break-after: always;"></div>

## 1.1. Installatie instructies <a name="chapter1"></a>


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

### 1.1.1. De applicatie gebruiken met launch script <a name="chapter2"></a>

1. Launch script (new terminal)

```bash
cd ~/wor_sim_review/
./launch.sh
```

<div style="page-break-after: always;"></div>


### 1.1.2. De applicatie gebruiken zonder launch script <a name="chapter3"></a>

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

<div style="page-break-after: always;"></div>

## 1.2. Bewegen van de robot <a name="chapter4"></a>

De virtuele robot kan op twee manieren bewogen worden. Er is een demonstratie script geschreven die verschillende commando's naar de virtuele controller stuurt en daarmee de robot beweegt.

Verder is het mogelijk om handmatig commando's naar de virtuele controller te sturen. Beide deze opties zijn in de onderstaande onderdelen uitgelegd.

### 1.2.1. Demo script <a name="chapter5"></a>

Er is een kleine demonstratie opgezet om de verschillende capaciteiten van de robot weer te geven.
Dit demo script zal onder andere het kopje oppakken en verplaatsen. Met het onderstaande commando kan het demo script uitgevoerd worden.

1. Demo script (new terminal)

```bash
cd ~/wor_sim_review/
. install/setup.bash
./demo.sh
```

### 1.2.2. Bewegen door middel van commando's <a name="chapter6"></a>

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

<div style="page-break-after: always;"></div>

Commando's publiceren naar het topic waar de arm naar luistert: ```/sim/controller/command``` kan gerealiseerd worden door middel van het volgende commando:

1. Nieuwe terminal (eenmalig)

```bash
cd ~/wor_sim_review/
. install/setup.bash
```

2. Versturen van de commando's.

```bash
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S500#1P1833S500#2P1444S500#3P722S500#4P500S500#5P1000S500'}"
```

Verschillende voorbeelden van bewegingen zijn terug te vinden in het demo script (/demo.sh)



