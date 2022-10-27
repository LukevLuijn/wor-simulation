# WOR - Simulatie

Uitwerking van het beroepsproduct World of Robots - World: Simulatie.

## TODO

- launch rviz from launch file.
- launch rqt from launch file (if possible).
- display arm urdf in rviz.
- write arm node code.
- write demo script.
- write design docs.
- write user guide doc.

- if possible do all the req's (time).

## Requirements

|  #   |  Prio  | Beschrijving                                                                                                                                                                                                                                                                    |      Behaald       |
|:----:|:------:|:--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:------------------:|
|  --  |   --   | **PACKAGE**                                                                                                                                                                                                                                                                     |         --         |
| PA01 | Should | Alle code is gepackaged volgens de ROS-directorystructuur.                                                                                                                                                                                                                      | :heavy_check_mark: |
| PA02 |  Must  | Package is te bouwen met colcon op Foxy Fitzroy                                                                                                                                                                                                                                 | :heavy_check_mark: |
| PA03 |  Must  | De applicatie wordt gebouwd met C++ volgens de Object Oriented principes die je geleerd hebt bij eerdere courses.                                                                                                                                                               | :heavy_check_mark: |
| PA04 | Should | Alle code voldoet aan de ROS C++ Style Guide.                                                                                                                                                                                                                                   | :heavy_check_mark: |
|  --  |   --   | **VIRTUELE SERVO CONTROLLER**                                                                                                                                                                                                                                                   |         --         |
| VS01 |  Must  | De virtuele controller luistert naar een topic waarop string messages in het formaat van de SSC-32U 1 worden geplaatst. Van de interface moeten ten minste commando’s zijn opgenomen voor het verplaatsen van de servo’s met een ingestelde duur en het stoppen van de servo’s. | :heavy_check_mark: |
| VS02 |  Must  | De virtuele controller reageert op het topic (zie eis VS01) door bijbehorende joint_state messages te publiceren.                                                                                                                                                               |        :x:         |
| VS03 |  Must  | De virtuele robotarm wordt gevisualiseerd in Rviz (een URDF-model van de arm is beschikbaar op OnderwijsOnline).                                                                                                                                                                |        :x:         |
| VS04 |  Must  | De virtuele robotarm gedraagt zich realistisch m.b.t. tijdgedrag (servo’s roteren kost tijd en gaat geleidelijk).                                                                                                                                                               |        :x:         |
| VS05 | Should | De virtuele robotarm kan op een willekeurige plaats in de virtuele wereld geplaatst worden.                                                                                                                                                                                     | :heavy_check_mark: |
|  --  |   --   | **VIRTUEEL BEKERTJE**                                                                                                                                                                                                                                                           |         --         |
| VC01 | Should | Er kan op een willekeurige plek in de virtuele wereld een bekertje geplaatst worden.                                                                                                                                                                                            | :heavy_check_mark: |
| VC02 |  Must  | Publiceert een 3D-visualisatie van het bekertje voor Rviz.                                                                                                                                                                                                                      | :heavy_check_mark: |
| VC03 | Should | Detecteert de relevante punten van de gripper.                                                                                                                                                                                                                                  |        :x:         |
| VC04 | Could  | Visualiseert de gedetecteerde punten van de gripper.                                                                                                                                                                                                                            |        :x:         |
| VC05 | Should | Visualiseert wanneer de gripper het bekertje vastheeft.                                                                                                                                                                                                                         |        :x:         |
| VC06 |  Must  | Het bekertje beweegt mee met de gripper (als hij vastgehouden wordt).                                                                                                                                                                                                           |        :x:         |
| VC07 |  Must  | Bekertje is onderhevig aan zwaartekracht wanneer losgelaten.                                                                                                                                                                                                                    | :heavy_check_mark: |
| VC08 |  Must  | Bekertje bepaalt en publiceert zijn positie.                                                                                                                                                                                                                                    | :heavy_check_mark: |
| VC09 | Should | Bekertje bepaalt en publiceert zijn snelheid.                                                                                                                                                                                                                                   | :heavy_check_mark: |
| VC10 | Could  | Snelheid wordt getoond met rqt_plot.                                                                                                                                                                                                                                            | :heavy_check_mark: |
|  --  |   --   | **DEMONSTRATIE-INFRASTRUCTUUR**                                                                                                                                                                                                                                                 |         --         |
| DI01 |  Must  | Een demoscript stuurt over de tijd een sequentie van commando’s naar de armcontroller.                                                                                                                                                                                          |        :x:         |
| DI02 | Could  | Locatie van het bekertje wordt in de roslaunch-configuratie bepaald.                                                                                                                                                                                                            | :heavy_check_mark: |
| DI03 | Could  | Locatie van de arm in de wereld wordt in de roslaunch-configuratie bepaald.                                                                                                                                                                                                     |        :x:         |
|  --  |   --   | **GEBRUIKSHANDLEIDING**                                                                                                                                                                                                                                                         |         --         |
| DM01 |  Must  | Beschrijft hoe de code gebouwd kan worden.                                                                                                                                                                                                                                      |        :x:         |
| DM02 |  Must  | Beschrijft stap voor stap hoe de arm bewogen kan worden middels enkele voorbeelden.                                                                                                                                                                                             |        :x:         |
| DM03 |  Must  | Beschrijft welke eisen gerealiseerd zijn. En geeft hierbij een (korte) toelichting.                                                                                                                                                                                             |        :x:         |
|  --  |   --   | **ONTWERPDOCUMENTATIE**                                                                                                                                                                                                                                                         |         --         |
| DD01 |  Must  | Beschrijft de structuur van de package (Nodes, topics, messages, et cetera).                                                                                                                                                                                                    |        :x:         |
| DD02 |  Must  | Beschrijft de structuur en samenhang van de broncode (class-diagrams, beschrijving, et cetera).                                                                                                                                                                                 |        :x:         |
| DD03 | Could  | Beschrijft hoe het gedrag van alle belangrijke componenten gerealiseerd is.                                                                                                                                                                                                     |        :x:         |
| DD04 | Should | Beschrijft de API van alle publieke interfaces.                                                                                                                                                                                                                                 |        :x:         |


