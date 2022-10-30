De virtuele robot kan op twee manieren bewogen worden. Er is een demonstratie script geschreven die verschillende commando's naar de virtuele controller stuurt en daarmee de robot beweegt.

Verder is het mogelijk om handmatig commando's naar de virtuele controller te sturen. Beide deze opties zijn in de onderstaande onderdelen uitgelegd.

### Demo script

Er is een kleine demonstratie opgezet om de verschillende capaciteiten van de robot weer te geven.
Dit demo script zal onder andere het kopje oppakken en verplaatsen. Met het onderstaande commando kan het demoscript uitgevoerd worden.

1. Demo script (new terminal)

```bash
cd ~/wor_sim_review/
. install/setup.bash
./demo.sh
```

### Bewegen door middel van commando's

De verschillende servo's in de AL5D robot kunnen bewogen worden door middel van seriële commando's.
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